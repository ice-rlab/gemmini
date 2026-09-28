package gemmini

import chisel3._
import chisel3.util._


class ScalingFactorReadReq(addrWidth: Int) extends Bundle {
  val addr = UInt(addrWidth.W) 
  val scaling_enable = Bool() 
}

class ScalingFactorReadResp(numRows: Int, numCols: Int) extends Bundle {  
  // one combined scale per tile column: numCols = 2*dim, so this is dim (was a hardcoded 16)
  val combined_scales = Vec(numCols / 2, UInt(36.W))
}

class ScalingFactorMemIO(addrWidth: Int, dataWidth: Int, numRows: Int, numCols: Int, meshRows:Int, tileRows: Int) extends Bundle {
  // writes happen to all interleaved banks for a line
  val scale_mem_write_w = Flipped(Decoupled(new ScalingFactorWriteReq(addrWidth, dataWidth)))
  val scale_mem_write_act = Flipped(Decoupled(new ScalingFactorWriteReq(addrWidth, dataWidth)))
  val read_req = Flipped(Decoupled(new ScalingFactorReadReq(7)))
  val read_resp = Decoupled(new ScalingFactorReadResp(numRows, numCols))
  val dataType = Input(UInt(2.W))
  val scaleMemCntl = Input(new ScalingFactorCntl(meshRows*tileRows)) // dummy output to match interface
  val counter_i = Input(UInt(16.W))
  val counter_j = Input(UInt(16.W))  
  val counter_k = Input(UInt(16.W))
  val i = Input(UInt(16.W))
  val j = Input(UInt(16.W))  
  val k = Input(UInt(16.W))
}

class ScalingFactorMem(
  depth: Int = 64,                     
  sramWidth: Int = 128,                 
  actOutputScalingWidth: Int = 8,       
  numBanks: Int = 8,
  testConfig: Boolean = false ,
  meshRows: Int,
  tileRows: Int,
) extends Module {
  val scaleMemSizeFactor = 4
  val doubleBufferFactor = 2
  val totalSizeBytes = 64*sramWidth*numBanks/8
  val bytesPerBank = sramWidth / 8        
  val AddrWidth = log2Ceil(totalSizeBytes)      
  val bankaddressWidth = log2Ceil(numBanks) 
  val totalScales = 32
  val counterWidth = log2Ceil(totalScales)  
  val io = IO(new ScalingFactorMemIO(
    AddrWidth, 
    64, 
    2*meshRows*tileRows,  // 32 rows (activation scales)
    2*meshRows*tileRows,   // 32 columns (weight scales)
    meshRows,
    tileRows,
  ))

  val initByte = 0x7e.U(8.W)
  val defaultRow = VecInit(Seq.fill(bytesPerBank)(initByte)) 
  val counter_i = io.counter_i
  val counter_j = io.counter_j
  val counter_k = io.counter_k
  val depth_sram = 64
  val half_sfMem_rows = depth_sram * numBanks / 2
  val bankDataT = Vec(bytesPerBank, UInt(8.W))
  val banks = Seq.fill(numBanks)(SyncReadMem(depth_sram, bankDataT))
  // Banks per scale type (weight / activation). The write decode below takes a 2-bit bank index and
  // the read side a one-hot table of {2 halves} x {double buffer}, so this is 4, not a free parameter.
  val banksPerType = numBanks / 2
  require(banksPerType == 4, s"ScalingFactorMem: address decode assumes 4 banks per scale type, got numBanks=$numBanks")

  val read_fire = io.read_req.fire && io.read_req.bits.scaling_enable
  val read_fire_d1 = RegNext(read_fire, false.B)

  // Tile geometry. This scale path was written for 16x16 processing-element tiles: the k-group
  // shift, the "one SRAM fetch per 16 output rows" cadence and the SRAM row/byte addressing were all
  // hardcoded for dim = 16. On a dim-8 Gemmini (RadianceSingleClusterSynConfig) FP8 GEMMs therefore
  // read scales for the wrong column groups and, on every other tile row, a stale row whose act
  // scales are 0 -- measured 2026-09-12 as whole-output corruption alternating every 8 rows.
  // Everything below is parameterized so that dim = 16 elaborates exactly as before.
  //   dim             elements per tile side = output rows written per tile = scales per fetch
  //   kGroupShift     log2(k-tiles per 32-element MX block) = log2(32 / dim)
  //   tilesPerRowLog  log2(PE tiles per 16-byte scale-SRAM row) = log2(16 / dim); software writes
  //                   the e8m0 scales in [K/32][M] byte order, 16 bytes per SRAM row
  val dim = meshRows * tileRows
  require(dim <= 16 && 16 % dim == 0, s"ScalingFactorMem: dim (meshRows*tileRows) must divide 16, got $dim")
  val lastWrite = dim - 1
  val kGroupShift = log2Ceil(32 / dim)
  val tilesPerRowLog = log2Ceil(16 / dim)
  val counter_i_runtime = RegInit(0.U(9.W))
  val counter_j_runtime = RegInit(0.U(9.W)) 
  val counter_k_runtime = RegInit(0.U(9.W))
  dontTouch(counter_i_runtime)
  dontTouch(counter_j_runtime)
  dontTouch(counter_k_runtime)
  val scale_counter = RegInit(0.U((log2Ceil(dim) max 1).W)) // counts reads 0..dim-1 (was 4 bits, dim 16)
  val scale_counter_d1 = RegNext(scale_counter)
  when(read_fire_d1) {
    when((counter_i_runtime === (io.scaleMemCntl.loop_bound_i - 1.U)) && 
        (scale_counter === lastWrite.U)) {
      counter_i_runtime := 0.U
    }.elsewhen(scale_counter === lastWrite.U) {
      counter_i_runtime := counter_i_runtime + 1.U
    }

    when((counter_j_runtime === (io.scaleMemCntl.loop_bound_j - 1.U)) && 
        (counter_i_runtime === (io.scaleMemCntl.loop_bound_i - 1.U)) && 
        (scale_counter === lastWrite.U)) {
      counter_j_runtime := 0.U
    }.elsewhen((counter_i_runtime === (io.scaleMemCntl.loop_bound_i - 1.U)) && 
              (scale_counter === lastWrite.U)) {
      counter_j_runtime := counter_j_runtime + 1.U
    }

    when((counter_j_runtime === (io.scaleMemCntl.loop_bound_j - 1.U)) && 
        (counter_i_runtime === (io.scaleMemCntl.loop_bound_i - 1.U)) && 
        (counter_k_runtime === (io.scaleMemCntl.loop_bound_k - 1.U)) && 
        (scale_counter === lastWrite.U)) {
      counter_k_runtime := 0.U
    }.elsewhen((counter_j_runtime === (io.scaleMemCntl.loop_bound_j - 1.U)) && 
              (counter_i_runtime === (io.scaleMemCntl.loop_bound_i - 1.U)) && 
              (scale_counter === lastWrite.U)) {
      counter_k_runtime := counter_k_runtime + 1.U
    }
  }
  


  val combined_scales_buffer = WireInit(VecInit(Seq.fill(2*meshRows*tileRows)(
  VecInit(Seq.fill(2*meshRows*tileRows)(0.U(9.W))))))
  val combined_scales_buffer_reg = RegInit(VecInit(Seq.fill(2*meshRows*tileRows)(
  VecInit(Seq.fill(2*meshRows*tileRows)(0.U(9.W))))))
  val combined_scales_valid = WireDefault(false.B)
  //val bankDataT = Vec(bytesPerBank, UInt(8.W))
  //val banks =RegInit(VecInit(Seq.fill(scaleMemSizeFactor*doubleBufferFactor*8*numBanks)(VecInit(Seq.fill(bytesPerBank)(0.U(8.W))))))

  val fp8Mode = io.dataType === 0.U
  
  val write_addr_w = io.scale_mem_write_w.bits.addr
  val write_weight_counter  = RegInit(0.U(2.W))
  val write_weight_full_row = RegInit(0.U(sramWidth.W))
  //val write_row_addr_w = WireInit(write_addr_w(log2Ceil(bytesPerBank) + log2Ceil(half_sfMem_rows) ,log2Ceil(bytesPerBank)))
  // FP8 mode
  val write_row_addr_w_fp8 = WireInit(write_addr_w(log2Ceil(bytesPerBank) + log2Ceil(depth_sram) - 1, log2Ceil(bytesPerBank)))
  val bank_idx_w_fp8 = WireInit(write_addr_w(log2Ceil(bytesPerBank) + log2Ceil(depth_sram) + 1, log2Ceil(bytesPerBank) + log2Ceil(depth_sram)))
  // Non-FP8 mode  
  val bank_idx_w_internal = WireInit(write_addr_w(log2Ceil(bytesPerBank)))
  val write_row_addr_w_nonfp8 = WireInit(write_addr_w(log2Ceil(bytesPerBank) + log2Ceil(depth_sram), log2Ceil(bytesPerBank) + 1))
  val bank_idx_w_nonfp8 = WireInit(write_addr_w(log2Ceil(bytesPerBank) + log2Ceil(depth_sram) + 2, log2Ceil(bytesPerBank) + log2Ceil(depth_sram) + 1))
  dontTouch(write_row_addr_w_fp8)
  dontTouch(bank_idx_w_fp8)
  dontTouch(bank_idx_w_internal)
  dontTouch(write_row_addr_w_nonfp8)
  dontTouch(bank_idx_w_nonfp8)

  when(write_weight_counter === 1.U && io.scale_mem_write_w.fire){
    write_weight_counter := 0.U
    val write_bytes = Cat(io.scale_mem_write_w.bits.data, write_weight_full_row(63, 0)).asTypeOf(bankDataT)
    val bank_sel_w = Mux(fp8Mode, bank_idx_w_fp8, Cat(bank_idx_w_nonfp8, bank_idx_w_internal))
    val write_row_addr_w = Mux(fp8Mode, write_row_addr_w_fp8, write_row_addr_w_nonfp8)
    for (b <- 0 until banksPerType) {
      when(bank_sel_w === b.U) {
        banks(b).write(write_row_addr_w, write_bytes)
      }
    }
  }.elsewhen(io.scale_mem_write_w.fire) {
      write_weight_counter := write_weight_counter + 1.U
      write_weight_full_row := Cat(write_weight_full_row(127, 64), io.scale_mem_write_w.bits.data) 
  }
  
  val write_addr_a = io.scale_mem_write_act.bits.addr
  val write_act_counter  = RegInit(0.U(2.W))
  val write_act_full_row = RegInit(0.U(sramWidth.W))
  // FP8 mode
  val write_row_addr_act_fp8 = WireInit(write_addr_a(log2Ceil(bytesPerBank) + log2Ceil(depth_sram) - 1, log2Ceil(bytesPerBank)))
  val bank_idx_act_fp8 = WireInit(write_addr_a(log2Ceil(bytesPerBank) + log2Ceil(depth_sram) + 1, log2Ceil(bytesPerBank) + log2Ceil(depth_sram)))
  // Non-FP8 mode
  val bank_idx_act_internal = WireInit(write_addr_a(log2Ceil(bytesPerBank)))
  val write_row_addr_act_nonfp8 = WireInit(write_addr_a(log2Ceil(bytesPerBank) + log2Ceil(depth_sram), log2Ceil(bytesPerBank) + 1))
  val bank_idx_act_nonfp8 = WireInit(write_addr_a(log2Ceil(bytesPerBank) + log2Ceil(depth_sram) + 2, log2Ceil(bytesPerBank) + log2Ceil(depth_sram) + 1))
  dontTouch(write_row_addr_act_fp8)
  dontTouch(bank_idx_act_fp8)
  dontTouch(bank_idx_act_internal)
  dontTouch(write_row_addr_act_nonfp8)
  dontTouch(bank_idx_act_nonfp8)
  when(write_act_counter === 1.U && io.scale_mem_write_act.fire) {
      write_act_counter := 0.U
      val write_bytes = Cat(io.scale_mem_write_act.bits.data, write_act_full_row(63, 0)).asTypeOf(bankDataT)
      val bank_sel_act = Mux(fp8Mode, bank_idx_act_fp8, Cat(bank_idx_act_nonfp8, bank_idx_act_internal))
      val write_row_addr_act = Mux(fp8Mode, write_row_addr_act_fp8, write_row_addr_act_nonfp8)
      for (b <- 0 until banksPerType) {
        when(bank_sel_act === b.U) {
          banks(b + banksPerType).write(write_row_addr_act, write_bytes)
        }
      }
  }.elsewhen(io.scale_mem_write_act.fire) {
      write_act_counter := write_act_counter + 1.U
      write_act_full_row := Cat(write_act_full_row(127, 64), io.scale_mem_write_act.bits.data)
  }


  val max_block_fp8 = meshRows * tileRows
  val max_block_non_fp8 = 2 * meshRows * tileRows

  // k: one MX block = 32 elements = 32/dim k-tiles -> group index = k >> kGroupShift.
  // FP8: one 16-byte SRAM row holds the per-element-row scales of 16/dim PE tiles, so the row is
  // (bound >> tilesPerRowLog) * group + (tile >> tilesPerRowLog), and the tile's scales start at
  // byte (tile % (16/dim)) * dim (act_byte_base_d1 / w_byte_base_d1 below). For dim = 16 this is
  // exactly the original bound * (k >> 1) + tile, byte 0. Requires the i/j loop bounds to be
  // multiples of 16/dim (i.e. M and N multiples of 16), which software's [K/32][M] layout implies.
  // FP4/FP6 keep the original row formula with the generalized group shift (validated at dim 16 only).
  val row_addr_width = log2Ceil(2*depth_sram)
  val read_row_addr_act = Wire(UInt(row_addr_width.W))
  val read_row_addr_w = Wire(UInt(row_addr_width.W))
  val k_group = counter_k_runtime >> kGroupShift
  if (tilesPerRowLog == 0) {
    read_row_addr_act := (io.scaleMemCntl.loop_bound_i) * k_group + (counter_i_runtime)
    read_row_addr_w := (io.scaleMemCntl.loop_bound_j) * k_group + (counter_j_runtime)
  } else {
    when(fp8Mode) {
      read_row_addr_act := (io.scaleMemCntl.loop_bound_i >> tilesPerRowLog) * k_group + (counter_i_runtime >> tilesPerRowLog)
      read_row_addr_w := (io.scaleMemCntl.loop_bound_j >> tilesPerRowLog) * k_group + (counter_j_runtime >> tilesPerRowLog)
    }.otherwise {
      read_row_addr_act := (io.scaleMemCntl.loop_bound_i) * k_group + (counter_i_runtime)
      read_row_addr_w := (io.scaleMemCntl.loop_bound_j) * k_group + (counter_j_runtime)
    }
  }
  val read_bank_idx_act = WireDefault(read_row_addr_act(row_addr_width - 1))
  val read_bank_idx_w = WireDefault(read_row_addr_w(row_addr_width - 1))
  val read_row_addr_act_real = WireDefault(read_row_addr_act(row_addr_width - 2, 0))
  val read_row_addr_w_real = WireDefault(read_row_addr_w(row_addr_width - 2, 0))
  val double_buffer_act_sel = io.scaleMemCntl.scale_mem_read_act_sel.asBool
  val double_buffer_w_sel = io.scaleMemCntl.scale_mem_read_w_sel.asBool

  dontTouch(read_row_addr_act)
  dontTouch(read_row_addr_w)
  dontTouch(read_bank_idx_act)
  dontTouch(read_bank_idx_w)
  dontTouch(read_row_addr_act_real)
  dontTouch(read_row_addr_w_real)

  io.scale_mem_write_w.ready := true.B
  io.scale_mem_write_act.ready := true.B

  val act_scales = WireDefault(VecInit(Seq.fill(meshRows*tileRows * 2)(0.U(8.W))))
  val weight_scales = WireDefault(VecInit(Seq.fill(meshRows*tileRows * 2)(0.U(8.W))))
  dontTouch(act_scales)
  dontTouch(weight_scales)
  
 

  def multiplyScalesE8M0(act: UInt, weight: UInt): UInt = {
    val sum = act +& weight
    sum(8, 0)
  }
  val read_fire_real = WireDefault(false.B) 
  val read_fire_real_d = RegNext(read_fire_real)
  
  read_fire_real := read_fire && (scale_counter === 0.U)

  // Byte offset of the current tile's scales inside the fetched 16-byte row, registered on the fetch
  // so it lines up with the SyncReadMem output one cycle later. Always 0 when dim = 16.
  val act_byte_base_d1 = RegInit(0.U(log2Ceil(bytesPerBank).W))
  val w_byte_base_d1 = RegInit(0.U(log2Ceil(bytesPerBank).W))
  if (tilesPerRowLog > 0) {
    when(read_fire_real) {
      act_byte_base_d1 := counter_i_runtime(tilesPerRowLog - 1, 0) << log2Ceil(dim)
      w_byte_base_d1 := counter_j_runtime(tilesPerRowLog - 1, 0) << log2Ceil(dim)
    }
  }
  def actByte(i: Int): UInt = if (tilesPerRowLog == 0) i.U else act_byte_base_d1 + i.U
  def wByte(i: Int): UInt = if (tilesPerRowLog == 0) i.U else w_byte_base_d1 + i.U 
  
  // val act_bank_data_vec = WireInit(VecInit(Seq.fill(meshRows*tileRows*2)(0.U(8.W))))
  // val weight_bank_data_vec = WireInit(VecInit(Seq.fill(meshRows*tileRows*2)(0.U(8.W))))
  // dontTouch(act_bank_data_vec)
  // dontTouch(weight_bank_data_vec)

  val read_fire_banks = VecInit(Seq(
    read_fire_real && (Mux(fp8Mode, (read_bank_idx_w === 0.U) && !double_buffer_w_sel   , !double_buffer_w_sel  )),  
    read_fire_real && (Mux(fp8Mode, (read_bank_idx_w === 1.U) && !double_buffer_w_sel   , !double_buffer_w_sel  )),  
    read_fire_real && (Mux(fp8Mode, (read_bank_idx_w === 0.U) && double_buffer_w_sel    , double_buffer_w_sel  )),  
    read_fire_real && (Mux(fp8Mode, (read_bank_idx_w === 1.U) && double_buffer_w_sel    , double_buffer_w_sel  )),  
    read_fire_real && (Mux(fp8Mode, (read_bank_idx_act === 0.U) && !double_buffer_act_sel , !double_buffer_act_sel)),
    read_fire_real && (Mux(fp8Mode, (read_bank_idx_act === 1.U) && !double_buffer_act_sel , !double_buffer_act_sel)),
    read_fire_real && (Mux(fp8Mode, (read_bank_idx_act === 0.U) && double_buffer_act_sel  , double_buffer_act_sel)),
    read_fire_real && (Mux(fp8Mode, (read_bank_idx_act === 1.U) && double_buffer_act_sel  , double_buffer_act_sel)),
  ))
  val read_fire_banks_d1 = RegNext(read_fire_banks)
  val bank_data_0 = VecInit((0 until banksPerType).map { i =>  banks(i).read(read_row_addr_w_real, read_fire_banks(i))})
  val bank_data_1 = VecInit((0 until banksPerType).map { i =>  banks(i+banksPerType).read(read_row_addr_act_real, read_fire_banks(i+banksPerType))})

  when(fp8Mode){
    // FP8: the tile's dim scales start at actByte(0) / wByte(0) inside the fetched row (see above).
    when(read_fire_banks_d1(4)) {
      for (i <- 0 until meshRows*tileRows) {
        act_scales(i) := bank_data_1(0)(actByte(i))
      }
    }.elsewhen(read_fire_banks_d1(5)) {
      for (i <- 0 until meshRows*tileRows) {
        act_scales(i) := bank_data_1(1)(actByte(i))
      }
    }.elsewhen(read_fire_banks_d1(6)) {
      for (i <- 0 until meshRows*tileRows) {
        act_scales(i) := bank_data_1(2)(actByte(i))
      }
    }.elsewhen(read_fire_banks_d1(7)) {
      for (i <- 0 until meshRows*tileRows) {
        act_scales(i) := bank_data_1(3)(actByte(i))
      }
    }
    when(read_fire_banks_d1(0)) {
      for (i <- 0 until meshRows*tileRows) {
        weight_scales(i) := bank_data_0(0)(wByte(i))
      }
    }.elsewhen(read_fire_banks_d1(1)) {
      for (i <- 0 until meshRows*tileRows) {
        weight_scales(i) := bank_data_0(1)(wByte(i))
      }
    }.elsewhen(read_fire_banks_d1(2)) {
      for (i <- 0 until meshRows*tileRows) {
        weight_scales(i) := bank_data_0(2)(wByte(i))
      }
    }.elsewhen(read_fire_banks_d1(3)) {
      for (i <- 0 until meshRows*tileRows) {
        weight_scales(i) := bank_data_0(3)(wByte(i))
      }
    }
  }.otherwise{
    when(read_fire_banks_d1(4)) {
      for (i <- 0 until meshRows*tileRows) {
        act_scales(i) := bank_data_1(0)(i)
        act_scales(meshRows*tileRows+i) := bank_data_1(1)(i)
      }
    }.elsewhen(read_fire_banks_d1(6)) {
      for (i <- 0 until meshRows*tileRows) {
        act_scales(i) := bank_data_1(2)(i)
        act_scales(meshRows*tileRows+i) := bank_data_1(3)(i)
      }
    }
    when(read_fire_banks_d1(0)) {
      for (i <- 0 until meshRows*tileRows) {
        weight_scales(i) := bank_data_0(0)(i)
        weight_scales(meshRows*tileRows+i) := bank_data_0(1)(i)
      }
    }.elsewhen(read_fire_banks_d1(2)) {
      for (i <- 0 until meshRows*tileRows) {
        weight_scales(i) := bank_data_0(2)(i)
        weight_scales(meshRows*tileRows+i) := bank_data_0(3)(i)
      }
    }
  }


  // val bank_data_0_act = VecInit((0 until 4).map { i => if (testConfig) defaultRow  else banks(i).read(read_row_addr_act, read_fire_banks(i))})
  // val bank_data_0_w = VecInit((0 until 4).map { i => if (testConfig) defaultRow  else banks(i+4).read(read_row_addr_w, read_fire_banks(i+4))})

  // when(fp8Mode){
  //   when(read_fire_d1){
  //     for (i <- 0 until meshRows*tileRows) {
  //       act_bank_data_vec(i) := bank_data_0_act(i)
  //     }
  //     for (i <- 0 until meshRows*tileRows) {
  //       weight_bank_data_vec(i) := bank_data_0_w(i)
  //     }
  //   }
  // }.otherwise{
  //   when(read_fire_d1) {
  //     for (i <- 0 until meshRows*tileRows) {
  //       act_bank_data_vec(i) := bank_data_0_act(i)
  //       act_bank_data_vec(meshRows*tileRows+i) := banks(read_row_addr_act + 1.U)(i)
  //     }
  //     for (i <- 0 until meshRows*tileRows) {
  //       weight_bank_data_vec(i) := bank_data_0_w(i)
  //       weight_bank_data_vec(meshRows*tileRows+i) := banks(read_row_addr_w + 1.U)(i)
  //     }
  //   }
  // }


  io.read_resp.bits.combined_scales.foreach(_ := 0.U)
  io.read_resp.valid := false.B
  io.read_req.ready := io.read_req.bits.scaling_enable 

  // for(i <- 0 until 2*meshRows*tileRows) {
  //   act_scales(i) := act_bank_data_vec(i)
  //   weight_scales(i) := weight_bank_data_vec(i)
  // }

  for(i <- 0 until 2*meshRows*tileRows) {     
    for(j <- 0 until 2*meshRows*tileRows) {
      when(read_fire_d1) {
        combined_scales_buffer(i)(j) := multiplyScalesE8M0(act_scales(i), weight_scales(j))
        combined_scales_valid := true.B  
      }.otherwise {
        combined_scales_buffer(i)(j) := 0.U
        combined_scales_valid := false.B  
      }
    }
  }

  when(read_fire) {
    when(scale_counter === lastWrite.U) {
      scale_counter := 0.U
    }.otherwise{
      scale_counter := scale_counter + 1.U
    }
  }

  when(read_fire_real_d){
    combined_scales_buffer_reg := combined_scales_buffer
  }

  when(read_fire_d1) {
    io.read_resp.valid := combined_scales_valid
    when(fp8Mode){
      when(scale_counter_d1 === 0.U){
        for(i <- 0 until meshRows*tileRows) {
          val single_scale = combined_scales_buffer(scale_counter_d1)(i)
          io.read_resp.bits.combined_scales(i) := Cat(0.U(27.W), single_scale(8, 0))
        }
      }.otherwise{
        for(i <- 0 until meshRows*tileRows) {
          val single_scale = combined_scales_buffer_reg(scale_counter_d1)(i)
          io.read_resp.bits.combined_scales(i) := Cat(0.U(27.W), single_scale(8, 0))
        }
      }
    }.otherwise{
      when(scale_counter_d1 === 0.U){
        for (i <- 0 until meshRows*tileRows) {
          val scale_row_0 = combined_scales_buffer((scale_counter_d1 << 1.U))
          val scale_row_1 = combined_scales_buffer((scale_counter_d1 << 1.U) + 1.U)
          val scale_all = Cat(scale_row_1.asUInt, scale_row_0.asUInt)
          io.read_resp.bits.combined_scales(i) := scale_all(36*i+35, 36*i)
        }
      }.otherwise{
        for (i <- 0 until meshRows*tileRows) {
          val scale_row_0 = combined_scales_buffer_reg((scale_counter_d1 << 1.U))      
          val scale_row_1 = combined_scales_buffer_reg((scale_counter_d1 << 1.U) + 1.U) 
          val scale_all = Cat(scale_row_1.asUInt, scale_row_0.asUInt)
          io.read_resp.bits.combined_scales(i) := scale_all(36*i+35, 36*i)
        }
      }
    }
  }
}
