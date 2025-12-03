package gemmini

import chisel3._
import org.chipsalliance.cde.config.{Config, Parameters}
import freechips.rocketchip.diplomacy.LazyModule
import freechips.rocketchip.subsystem._
import freechips.rocketchip.tile.{BuildRoCC, OpcodeSet}
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._
import freechips.rocketchip.system._
import freechips.rocketchip.diplomacy._

import gemmini.Arithmetic.SIntArithmetic
import hardfloat._

// -----------------------
// Component Mixin Configs
// -----------------------

object GemminiConfigs {
  val defaultConfig = GemminiArrayConfig[SInt, Float, Float](
    // Datatypes
    inputType = SInt(8.W),
    weightType = SInt(8.W),
    accType = SInt(32.W),

    spatialArrayInputType = SInt(8.W),
    spatialArrayWeightType = SInt(8.W),
    spatialArrayOutputType = SInt(20.W),

    // Spatial array size options
    tileRows = 1,
    tileColumns = 1,
    meshRows = 16,
    meshColumns = 16,

    // Spatial array PE options
    dataflow = Dataflow.BOTH,

    // Scratchpad and accumulator
    sp_capacity = CapacityInKilobytes(256),
    acc_capacity = CapacityInKilobytes(64),

    sp_banks = 4,
    acc_banks = 2,

    sp_singleported = true,
    acc_singleported = false,

    // DNN options
    has_training_convs = true,
    has_max_pool = true,
    has_nonlinear_activations = true,

    // Reservation station entries
    reservation_station_entries_ld = 8,
    reservation_station_entries_st = 4,
    reservation_station_entries_ex = 16,

    // Ld/Ex/St instruction queue lengths
    ld_queue_length = 8,
    st_queue_length = 2,
    ex_queue_length = 8,

    // DMA options
    max_in_flight_mem_reqs = 16,

    dma_maxbytes = 64,
    dma_buswidth = 128,

    // TLB options
    tlb_size = 4,

    // Mvin and Accumulator scalar multiply options
    mvin_scale_args = Some(ProfilingScaleArguments(
      (t: SInt, f: Float) => {
        val f_rec = recFNFromFN(f.expWidth, f.sigWidth, f.bits)

        val in_to_rec_fn = Module(new INToRecFN(t.getWidth, f.expWidth, f.sigWidth))
        in_to_rec_fn.io.signedIn := true.B
        in_to_rec_fn.io.in := t.asTypeOf(UInt(t.getWidth.W))
        in_to_rec_fn.io.roundingMode := consts.round_near_even
        in_to_rec_fn.io.detectTininess := consts.tininess_afterRounding

        val t_rec = in_to_rec_fn.io.out

        val muladder = Module(new MulAddRecFN(f.expWidth, f.sigWidth))
        muladder.io.op := 0.U
        muladder.io.roundingMode := consts.round_near_even
        muladder.io.detectTininess := consts.tininess_afterRounding

        muladder.io.a := t_rec
        muladder.io.b := f_rec
        muladder.io.c := 0.U

        val rec_fn_to_in = Module(new RecFNToIN(f.expWidth, f.sigWidth, t.getWidth))
        rec_fn_to_in.io.in := muladder.io.out
        rec_fn_to_in.io.roundingMode := consts.round_near_even
        rec_fn_to_in.io.signedOut := true.B

        val overflow = rec_fn_to_in.io.intExceptionFlags(1)
        val maxsat = ((1 << (t.getWidth-1))-1).S
        val minsat = (-(1 << (t.getWidth-1))).S
        val sign = rawFloatFromRecFN(f.expWidth, f.sigWidth, rec_fn_to_in.io.in).sign
        val sat = Mux(sign, minsat, maxsat)

        //Mux(overflow, sat, rec_fn_to_in.io.out.asTypeOf(t)) old return value

	val scaled_result = Mux(overflow, sat, rec_fn_to_in.io.out.asTypeOf(t))

	val profiling_val = 0.U.asTypeOf(f)

	val out = Wire(new ScaleFuncOutputs(t, f))
	out.result := scaled_result
	out.profiling := profiling_val

	out
      },
      4, Float(8, 24), 4,
      identity = "1.0",
      c_str = "({float y = ROUND_NEAR_EVEN((x) * (scale)); y > INT8_MAX ? INT8_MAX : (y < INT8_MIN ? INT8_MIN : (elem_t)y);})"
    )),

    mvin_scale_acc_args = None,
    mvin_scale_shared = false,

    acc_scale_args = Some(ScaleArguments(
      (t: SInt, f: Float) => {
        val f_rec = recFNFromFN(f.expWidth, f.sigWidth, f.bits)

        val in_to_rec_fn = Module(new INToRecFN(t.getWidth, f.expWidth, f.sigWidth))
        in_to_rec_fn.io.signedIn := true.B
        in_to_rec_fn.io.in := t.asTypeOf(UInt(t.getWidth.W))
        in_to_rec_fn.io.roundingMode := consts.round_near_even
        in_to_rec_fn.io.detectTininess := consts.tininess_afterRounding

        val t_rec = in_to_rec_fn.io.out

        val muladder = Module(new MulAddRecFN(f.expWidth, f.sigWidth))
        muladder.io.op := 0.U
        muladder.io.roundingMode := consts.round_near_even
        muladder.io.detectTininess := consts.tininess_afterRounding

        muladder.io.a := t_rec
        muladder.io.b := f_rec
        muladder.io.c := 0.U

        val rec_fn_to_in = Module(new RecFNToIN(f.expWidth, f.sigWidth, t.getWidth))
        rec_fn_to_in.io.in := muladder.io.out
        rec_fn_to_in.io.roundingMode := consts.round_near_even
        rec_fn_to_in.io.signedOut := true.B

        val overflow = rec_fn_to_in.io.intExceptionFlags(1)
        val maxsat = ((1 << (t.getWidth-1))-1).S
        val minsat = (-(1 << (t.getWidth-1))).S
        val sign = rawFloatFromRecFN(f.expWidth, f.sigWidth, rec_fn_to_in.io.in).sign
        val sat = Mux(sign, minsat, maxsat)

        Mux(overflow, sat, rec_fn_to_in.io.out.asTypeOf(t))
      },
      8, Float(8, 24), -1,
      identity = "1.0",
      c_str = "({float y = ROUND_NEAR_EVEN((x) * (scale)); y > INT8_MAX ? INT8_MAX : (y < INT8_MIN ? INT8_MIN : (acc_t)y);})"
    )),

    // SoC counters options
    num_counter = 8,

    // Scratchpad and Accumulator input/output options
    acc_read_full_width = true,
    acc_read_small_width = true,

    ex_read_from_spad = true,
    ex_read_from_acc = true,
    ex_write_to_spad = true,
    ex_write_to_acc = true,
  )

  val dummyConfig = GemminiArrayConfig[DummySInt, Float, Float](
    inputType = DummySInt(8),
    weightType = DummySInt(8),
    accType = DummySInt(32),
    spatialArrayInputType = DummySInt(8),
    spatialArrayWeightType = DummySInt(8),
    spatialArrayOutputType = DummySInt(20),
    tileRows     = defaultConfig.tileRows,
    tileColumns  = defaultConfig.tileColumns,
    meshRows     = defaultConfig.meshRows,
    meshColumns  = defaultConfig.meshColumns,
    dataflow     = defaultConfig.dataflow,
    sp_capacity  = CapacityInKilobytes(128),
    acc_capacity = CapacityInKilobytes(128),
    sp_banks     = defaultConfig.sp_banks,
    acc_banks    = defaultConfig.acc_banks,
    sp_singleported = defaultConfig.sp_singleported,
    acc_singleported = defaultConfig.acc_singleported,
    has_training_convs = false,
    has_max_pool = defaultConfig.has_max_pool,
    has_nonlinear_activations = false,
    reservation_station_entries_ld = defaultConfig.reservation_station_entries_ld,
    reservation_station_entries_st = defaultConfig.reservation_station_entries_st,
    reservation_station_entries_ex = defaultConfig.reservation_station_entries_ex,
    ld_queue_length = defaultConfig.ld_queue_length,
    st_queue_length = defaultConfig.st_queue_length,
    ex_queue_length = defaultConfig.ex_queue_length,
    max_in_flight_mem_reqs = defaultConfig.max_in_flight_mem_reqs,
    dma_maxbytes = defaultConfig.dma_maxbytes,
    dma_buswidth = defaultConfig.dma_buswidth,
    tlb_size = defaultConfig.tlb_size,

    mvin_scale_args = Some(ProfilingScaleArguments(
      (t: DummySInt, f: Float) => {
	val t_out = t.dontCare
	//val profiling_out = f.dontCare
	val profiling_out = 0.U.asTypeOf(f)

	val out = Wire(new ScaleFuncOutputs(t,f))
	out.result := t_out
	out.profiling := profiling_out

	out
	},
      4, Float(8, 24), 4,
      identity = "1.0",
      c_str = "({float y = ROUND_NEAR_EVEN((x) * (scale)); y > INT8_MAX ? INT8_MAX : (y < INT8_MIN ? INT8_MIN : (elem_t)y);})"
    )),

    mvin_scale_acc_args = None,
    mvin_scale_shared = defaultConfig.mvin_scale_shared,

    acc_scale_args = Some(ScaleArguments(
      (t: DummySInt, f: Float) => t.dontCare,
      1, Float(8, 24), -1,
      identity = "1.0",
      c_str = "({float y = ROUND_NEAR_EVEN((x) * (scale)); y > INT8_MAX ? INT8_MAX : (y < INT8_MIN ? INT8_MIN : (acc_t)y);})"
    )),

    num_counter = 0,

    acc_read_full_width = false,
    acc_read_small_width = defaultConfig.acc_read_small_width,

    ex_read_from_spad = defaultConfig.ex_read_from_spad,
    ex_read_from_acc = false,
    ex_write_to_spad = false,
    ex_write_to_acc = defaultConfig.ex_write_to_acc,
  )

// val countersFloat2IntConfig = GemminiArrayConfig[SInt, Float, Float, Float](
//     // Datatypes

//     scaleDownType = Float(8, 24),

//     inputType = SInt(8.W),
//     weightType = SInt(8.W),
//     accType = SInt(32.W),

//     spatialArrayInputType = SInt(8.W),
//     spatialArrayWeightType = SInt(8.W),
//     spatialArrayOutputType = SInt(20.W),

//     // Spatial array size options
//     tileRows = 1,
//     tileColumns = 1,
//     meshRows = 16,
//     meshColumns = 16,

//     // Spatial array PE options
//     dataflow = Dataflow.BOTH,

//     // Scratchpad and accumulator
//     sp_capacity = CapacityInKilobytes(256),
//     acc_capacity = CapacityInKilobytes(64),

//     sp_banks = 4,
//     acc_banks = 2,

//     sp_singleported = true,
//     acc_singleported = false,

//     // DNN options
//     has_training_convs = true,
//     has_max_pool = true,
//     has_nonlinear_activations = true,

//     // Reservation station entries
//     reservation_station_entries_ld = 8,
//     reservation_station_entries_st = 4,
//     reservation_station_entries_ex = 16,

//     // Ld/Ex/St instruction queue lengths
//     ld_queue_length = 8,
//     st_queue_length = 2,
//     ex_queue_length = 8,

//     // DMA options
//     max_in_flight_mem_reqs = 16,

//     dma_maxbytes = 64,
//     dma_buswidth = 128,

//     // TLB options
//     tlb_size = 4,

//     // Mvin and Accumulator scalar multiply options
//     mvin_scale_args = Some(ScaleArguments(
//       (t: Float, f: Float) => {

// 	//val fn_to_rec_rn = Module(new RecFNFromFN(t.expWidth, t.sigWidth, t.bits))
// 	val t_rec = recFNFromFN(f.expWidth, f.sigWidth, t.bits)
// 	//fn_to_rec_rn.io.in := t
//         val rec_fn_to_in = Module(new RecFNToIN(f.expWidth, f.sigWidth, t.getWidth))
//         rec_fn_to_in.io.in := t_rec
//         rec_fn_to_in.io.roundingMode := consts.round_near_even
//         rec_fn_to_in.io.signedOut := true.B

//         val overflow = rec_fn_to_in.io.intExceptionFlags(1)
//         val maxsat = ((1 << (t.getWidth-1))-1).S
//         val minsat = (-(1 << (t.getWidth-1))).S
//         val sign = rawFloatFromRecFN(f.expWidth, f.sigWidth, t_rec).sign
//         val sat = Mux(sign, minsat, maxsat)

//         Mux(overflow, sat, rec_fn_to_in.io.out.asSInt)
//       },
//       4, Float(8, 24), 4,
//       identity = "1.0",
//       c_str = "({float y = ROUND_NEAR_EVEN((x) * (scale)); y > INT8_MAX ? INT8_MAX : (y < INT8_MIN ? INT8_MIN : (elem_t)y);})"
//     )),

//     mvin_scale_acc_args = None,
//     mvin_scale_shared = false,

//     acc_scale_args = None, //bring this back?

//     // SoC counters options
//     num_counter = 255,

//     // Scratchpad and Accumulator input/output options
//     acc_read_full_width = true,
//     acc_read_small_width = true,

//     ex_read_from_spad = true,
//     ex_read_from_acc = true,
//     ex_write_to_spad = true,
//     ex_write_to_acc = true,
//   )

  val chipConfig = defaultConfig.copy(sp_capacity=CapacityInKilobytes(64), acc_capacity=CapacityInKilobytes(32), dataflow=Dataflow.WS,
    acc_scale_args=Some(defaultConfig.acc_scale_args.get.copy(latency=4)),
    acc_singleported=true,
    acc_sub_banks=2,
    mesh_output_delay = 2,
    ex_read_from_acc=false,
    ex_write_to_spad=false,
    hardcode_d_to_garbage_addr = true
  )

  val largeChipConfig = chipConfig.copy(sp_capacity=CapacityInKilobytes(128), acc_capacity=CapacityInKilobytes(64),
    tileRows=1, tileColumns=1,
    meshRows=32, meshColumns=32
  )

  val leanConfig = defaultConfig.copy(dataflow=Dataflow.WS, max_in_flight_mem_reqs = 64, acc_read_full_width = false, ex_read_from_acc = false, ex_write_to_spad = false, hardcode_d_to_garbage_addr = true)

  val leanPrintfConfig = defaultConfig.copy(dataflow=Dataflow.WS, max_in_flight_mem_reqs = 64, acc_read_full_width = false, ex_read_from_acc = false, ex_write_to_spad = false, hardcode_d_to_garbage_addr = true, use_firesim_simulation_counters=true)

  val twoKbSpadConfig = defaultConfig.copy(sp_capacity=CapacityInKilobytes(2), acc_capacity=CapacityInKilobytes(2), //trying to use same number of sp and acc banks as normal design, one bank not supported
    num_counter = 64
  )

  val twoKbSpad16BitInputConfig = defaultConfig.copy(sp_capacity=CapacityInKilobytes(2), acc_capacity=CapacityInKilobytes(2), //trying to use same number of sp and acc banks as normal design, one bank not supported
    num_counter = 64, inputType = SInt(16.W),
    weightType = SInt(16.W), spatialArrayInputType = SInt(16.W),
    spatialArrayWeightType = SInt(16.W),
  )

  val countersConfig = defaultConfig.copy(use_firesim_simulation_counters=true, //trying to use same number of sp and acc banks as normal design, one bank not supported
    num_counter = 255
  )

//   val countersFloat2IntConfig = defaultConfig.copy(use_firesim_simulation_counters=true, //trying to use same number of sp and acc banks as normal design, one bank not supported
//     num_counter = 255,
//     scaleDownType = Float(8, 24),
//     mvin_scale_args = Some(ScaleArguments(
//       (t: Float, f: Float) => {

// 	//val fn_to_rec_rn = Module(new RecFNFromFN(t.expWidth, t.sigWidth, t.bits))
// 	val t_rec = recFNFromFN(f.expWidth, f.sigWidth, t.bits)
// 	//fn_to_rec_rn.io.in := t
//         val rec_fn_to_in = Module(new RecFNToIN(f.expWidth, f.sigWidth, t.getWidth))
//         rec_fn_to_in.io.in := t_rec
//         rec_fn_to_in.io.roundingMode := consts.round_near_even
//         rec_fn_to_in.io.signedOut := true.B

//         val overflow = rec_fn_to_in.io.intExceptionFlags(1)
//         val maxsat = ((1 << (t.getWidth-1))-1).S
//         val minsat = (-(1 << (t.getWidth-1))).S
//         val sign = rawFloatFromRecFN(f.expWidth, f.sigWidth, t_rec).sign
//         val sat = Mux(sign, minsat, maxsat)

//         Mux(overflow, sat, rec_fn_to_in.io.out.asSInt)
//       },
//       4, Float(8, 24), 4,
//       identity = "1.0",
//       c_str = "({float y = ROUND_NEAR_EVEN((x) * (scale)); y > INT8_MAX ? INT8_MAX : (y < INT8_MIN ? INT8_MIN : (elem_t)y);})"
//     )),
//   )

  val oneCombinationalArrayConfig = defaultConfig.copy(use_firesim_simulation_counters=true, //trying to use same number of sp and acc banks as normal design, one bank not supported
    num_counter = 64, tileRows = 16,
    tileColumns = 16,
    meshRows = 1,
    meshColumns = 1
  )

  val shiftScaleConfig = defaultConfig.copy(use_firesim_simulation_counters=true, //trying to use same number of sp and acc banks as normal design, one bank not supported
    num_counter = 64,
    mvin_scale_args = Some(ProfilingScaleArguments(
  	(t: SInt, scale: Float) => {
		val t_out = (t >> 4).asTypeOf(t)
		val profiling_out = 0.U.asTypeOf(scale)

		val out = Wire(new ScaleFuncOutputs(t, scale))
		out.result := t_out
		out.profiling := profiling_out

		out
	},
  	4, Float(8, 24), -1,
  	identity = "1.0",
  	c_str = "(x >> 4)"
	))
// 	mvin_scale_args = Some(ScaleArguments(
//       (t: Float, f: Float) => t.dontCare,
//       4, Float(8, 24), 4,
//       identity = "1.0",
//       c_str = "({float y = ROUND_NEAR_EVEN((x) * (scale)); y > INT8_MAX ? INT8_MAX : (y < INT8_MIN ? INT8_MIN : (elem_t)y);})"
//     )),
  )
}

/**
 * Mixin which sets the default parameters for a systolic array accelerator.
   Also sets the system bus width to 128 bits (instead of the deafult 64 bits) to
   allow for the default 16x16 8-bit systolic array to be attached.
 */
class DefaultGemminiConfig[T <: Data : Arithmetic, U <: Data, V <: Data](
  gemminiConfig: GemminiArrayConfig[T,U,V] = GemminiConfigs.defaultConfig
) extends Config((site, here, up) => {
  case BuildRoCC => up(BuildRoCC) ++ Seq(
    (p: Parameters) => {
      implicit val q = p
      val gemmini = LazyModule(new Gemmini(gemminiConfig))
      gemmini
    }
  )
})

// class SimpleDualGemminiConfig[T <: Data : Arithmetic, U <: Data, V <: Data](
//   gemminiConfig: GemminiArrayConfig[T,U,V] = GemminiConfigs.countersConfig
//   gemminiFloatConfig: GemminiArrayConfig[T,U,V] = GemminiFPConfigs.FP32CountersConfig
// ) extends Config((site, here, up) => {
//   case BuildRoCC => up(BuildRoCC) ++ Seq(
//     (p: Parameters) => {
//       implicit val q = p
//       val gemmini = LazyModule(new Gemmini(gemminiConfig))
//       gemmini
//     },
//     (p: Parameters) => {
//       implicit val q = p
//       val gemminiFP = LazyModule(new Gemmini(gemminiFloatConfig))
//       gemminiFP
//     }
//   )
// })

// class CountersFloat2IntGemminiConfig[T <: Data : Arithmetic, U <: Data, V <: Data](
//   gemminiConfig: GemminiArrayConfig[T,U,V] = GemminiConfigs.countersFloat2IntConfig
// ) extends Config((site, here, up) => {
//   case BuildRoCC => up(BuildRoCC) ++ Seq(
//     (p: Parameters) => {
//       implicit val q = p
//       val gemmini = LazyModule(new Gemmini(gemminiConfig))
//       gemmini
//     }
//   )
// })

// class SimpleDualScalingGemminiConfig[T <: Data : Arithmetic, U <: Data, V <: Data](
//   gemminiConfig: GemminiArrayConfig[T,U,V] = GemminiConfigs.countersFloat2IntConfig
//   gemminiFloatConfig: GemminiArrayConfig[T,U,V] = GemminiFPConfigs.FP32CountersConfig
// ) extends Config((site, here, up) => {
//   case BuildRoCC => up(BuildRoCC) ++ Seq(
//     (p: Parameters) => {
//       implicit val q = p
//       val gemmini = LazyModule(new Gemmini(gemminiConfig))
//       gemmini
//     },
//     (p: Parameters) => {
//       implicit val q = p
//       val gemminiFP = LazyModule(new Gemmini(gemminiFloatConfig))
//       gemminiFP
//     }
//   )
// })

// class SimpleDualGemminiConfig extends Config((site, here, up) => {
//   case BuildRoCC => {
//     var int_gemmini: Gemmini[_,_,_] = null
//     var fp_gemmini: Gemmini[_,_,_] = null
//     val int_fn = (p: Parameters) => {
//       implicit val q = p
//       int_gemmini = LazyModule(new Gemmini(GemminiConfigs.countersConfig))
//       int_gemmini
//       }
//     val fp_fn = (p: Parameters) => {
//       implicit val q = p
//       fp_gemmini = LazyModule(new Gemmini(GemminiFPConfigs.FP32CountersConfig))
//       fp_gemmini
//     }
//     up(BuildRoCC) ++ Seq(int_fn, fp_fn)
//   }
// })

class CountersGemminiConfig[T <: Data : Arithmetic, U <: Data, V <: Data](
  gemminiConfig: GemminiArrayConfig[T,U,V] = GemminiConfigs.countersConfig
) extends Config((site, here, up) => {
  case BuildRoCC => up(BuildRoCC) ++ Seq(
    (p: Parameters) => {
      implicit val q = p
      val gemmini = LazyModule(new Gemmini(gemminiConfig))
      gemmini
    }
  )
})

class OneCombinationalArrayGemminiConfig[T <: Data : Arithmetic, U <: Data, V <: Data](
  gemminiConfig: GemminiArrayConfig[T,U,V] = GemminiConfigs.oneCombinationalArrayConfig
) extends Config((site, here, up) => {
  case BuildRoCC => up(BuildRoCC) ++ Seq(
    (p: Parameters) => {
      implicit val q = p
      val gemmini = LazyModule(new Gemmini(gemminiConfig))
      gemmini
    }
  )
})

//2kB spad, 2kB acc, 64 counters, 4 sp banks, 2 acc banks, datatypes
class TwoKbSpadGemminiConfig[T <: Data : Arithmetic, U <: Data, V <: Data](
  gemminiConfig: GemminiArrayConfig[T,U,V] = GemminiConfigs.twoKbSpadConfig
) extends Config((site, here, up) => {
  case BuildRoCC => up(BuildRoCC) ++ Seq(
    (p: Parameters) => {
      implicit val q = p
      val gemmini = LazyModule(new Gemmini(gemminiConfig))
      gemmini
    }
  )
})
//twoKbSpad16BitInputConfig

class TwoKbSpad16BitInputGemminiConfig[T <: Data : Arithmetic, U <: Data, V <: Data](
  gemminiConfig: GemminiArrayConfig[T,U,V] = GemminiConfigs.twoKbSpad16BitInputConfig
) extends Config((site, here, up) => {
  case BuildRoCC => up(BuildRoCC) ++ Seq(
    (p: Parameters) => {
      implicit val q = p
      val gemmini = LazyModule(new Gemmini(gemminiConfig))
      gemmini
    }
  )
})

class ShiftScaleGemminiConfig[T <: Data : Arithmetic, U <: Data, V <: Data](
  gemminiConfig: GemminiArrayConfig[T,U,V] = GemminiConfigs.shiftScaleConfig
) extends Config((site, here, up) => {
  case BuildRoCC => up(BuildRoCC) ++ Seq(
    (p: Parameters) => {
      implicit val q = p
      val gemmini = LazyModule(new Gemmini(gemminiConfig))
      gemmini
    }
  )
})

/**
 * Mixin which sets the default lean parameters for a systolic array accelerator.
 */
class LeanGemminiConfig[T <: Data : Arithmetic, U <: Data, V <: Data](
  gemminiConfig: GemminiArrayConfig[T,U,V] = GemminiConfigs.leanConfig
) extends Config((site, here, up) => {
  case BuildRoCC => up(BuildRoCC) ++ Seq(
    (p: Parameters) => {
      implicit val q = p
      val gemmini = LazyModule(new Gemmini(gemminiConfig))
      gemmini
    }
  )
})

class LeanGemminiPrintfConfig[T <: Data : Arithmetic, U <: Data, V <: Data](
  gemminiConfig: GemminiArrayConfig[T,U,V] = GemminiConfigs.leanPrintfConfig
) extends Config((site, here, up) => {
  case BuildRoCC => up(BuildRoCC) ++ Seq(
    (p: Parameters) => {
      implicit val q = p
      val gemmini = LazyModule(new Gemmini(gemminiConfig))
      gemmini
    }
  )
})

class DummyDefaultGemminiConfig[T <: Data : Arithmetic, U <: Data, V <: Data](
  gemminiConfig: GemminiArrayConfig[T,U,V] = GemminiConfigs.dummyConfig
) extends Config((site, here, up) => {
  case BuildRoCC => up(BuildRoCC) ++ Seq(
    (p: Parameters) => {
      implicit val q = p
      val gemmini = LazyModule(new Gemmini(gemminiConfig))
      gemmini
    }
  )
})

// This Gemmini config has both an Int and an FP Gemmini side-by-side, sharing
// the same scratchpad.
// class DualGemminiConfig extends Config((site, here, up) => {
//   case BuildRoCC => {
//     var int_gemmini: Gemmini[_,_,_] = null
//     var fp_gemmini: Gemmini[_,_,_] = null
//     val int_fn = (p: Parameters) => {
//       implicit val q = p
//       int_gemmini = LazyModule(new Gemmini(GemminiConfigs.chipConfig.copy(
//         opcodes = OpcodeSet.custom3,
//         use_shared_ext_mem = true,
//         clock_gate = true
//       )))
//       int_gemmini
//     }
//     val fp_fn = (p: Parameters) => {
//       implicit val q = p
//       fp_gemmini = LazyModule(new Gemmini(GemminiFPConfigs.BF16DefaultConfig.copy(
//         opcodes = OpcodeSet.custom2,
//         sp_capacity=CapacityInKilobytes(64), acc_capacity=CapacityInKilobytes(32),
//         tileColumns = 1, tileRows = 1,
//         meshColumns = 8, meshRows = 8,
//         acc_singleported = true, acc_banks = 2, acc_sub_banks = 2,
//         use_shared_ext_mem = true,
//         ex_read_from_acc=false,
//         ex_write_to_spad=false,
//         hardcode_d_to_garbage_addr = true,
//         headerFileName = "gemmini_params_bf16.h",
//         acc_latency = 3,
//         dataflow = Dataflow.WS,
//         mesh_output_delay = 3,
//         clock_gate = true
//       )))
//       InModuleBody {
//         require(int_gemmini.config.sp_banks == fp_gemmini.config.sp_banks)
//         require(int_gemmini.config.acc_banks == fp_gemmini.config.acc_banks)
//         require(int_gemmini.config.acc_sub_banks == fp_gemmini.config.acc_sub_banks)
//         require(int_gemmini.config.sp_singleported && fp_gemmini.config.sp_singleported)
//         require(int_gemmini.config.acc_singleported && fp_gemmini.config.acc_singleported)

//         require(int_gemmini.config.sp_bank_entries == fp_gemmini.config.sp_bank_entries)
//         require(int_gemmini.spad.module.spad_mems(0).mask_len == fp_gemmini.spad.module.spad_mems(0).mask_len)
//         require(int_gemmini.spad.module.spad_mems(0).mask_elem.getWidth == fp_gemmini.spad.module.spad_mems(0).mask_elem.getWidth)

//         println(int_gemmini.config.acc_bank_entries, fp_gemmini.config.acc_bank_entries)
//         println(int_gemmini.spad.module.acc_mems(0).mask_len, fp_gemmini.spad.module.acc_mems(0).mask_len)
//         println(int_gemmini.spad.module.acc_mems(0).mask_elem.getWidth, fp_gemmini.spad.module.acc_mems(0).mask_elem.getWidth)

//         require(int_gemmini.config.acc_bank_entries == fp_gemmini.config.acc_bank_entries / 2)
//         require(int_gemmini.config.acc_sub_banks == fp_gemmini.config.acc_sub_banks)
//         require(int_gemmini.spad.module.acc_mems(0).mask_len == fp_gemmini.spad.module.acc_mems(0).mask_len * 2)
//         require(int_gemmini.spad.module.acc_mems(0).mask_elem.getWidth == fp_gemmini.spad.module.acc_mems(0).mask_elem.getWidth)

//         val spad_mask_len = int_gemmini.spad.module.spad_mems(0).mask_len
//         val spad_data_len = int_gemmini.spad.module.spad_mems(0).mask_elem.getWidth
//         val acc_mask_len = int_gemmini.spad.module.acc_mems(0).mask_len
//         val acc_data_len = int_gemmini.spad.module.acc_mems(0).mask_elem.getWidth

//         val shared_mem = Module(new SharedExtMem(
//           int_gemmini.config.sp_banks, int_gemmini.config.acc_banks, int_gemmini.config.acc_sub_banks,
//           int_gemmini.config.sp_bank_entries, spad_mask_len, spad_data_len,
//           int_gemmini.config.acc_bank_entries / int_gemmini.config.acc_sub_banks, acc_mask_len, acc_data_len
//         ))
//         shared_mem.io.in(0) <> int_gemmini.module.ext_mem_io.get
//         shared_mem.io.in(1) <> fp_gemmini.module.ext_mem_io.get
//       }
//       fp_gemmini
//     }
//     up(BuildRoCC) ++ Seq(int_fn, fp_fn)
//   }
// })
