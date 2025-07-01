// Copyright 2017 MPI-SWS, Saarbruecken, Germany

package daisy
package analysis

import lang.Trees._
import lang.Identifiers._
import lang.Types.RealType
import tools._
import FinitePrecision._
import lang.TreeOps.allIDsOf

import daisy.OverflowFixException


/**
  Computes and stores intermediate ranges.

  Prerequisites:
    - SpecsProcessingPhase
 */
object DataflowBackupPhase extends DaisyPhase with RoundoffEvaluators with IntervalSubdivision with opt.CostFunctions {
  override val name = "Dataflow error"
  override val description = "Computes ranges and absolute errors via dataflow analysis"

  override val definedOptions: Set[CmdLineOption[Any]] = Set(
    StringChoiceOption(
      "errorMethod",
      Set("affine", "interval", "intervalMPFR", "affineMPFR"),
      "affine",
      "Method for error analysis"),
    StringChoiceOption(
      "choosePrecision",
      Set("no", "fixed", "float"),
      "no",
      "choose the fixed/floating-point precision which satisfies error bound")
  )
  override implicit val debugSection = DebugSectionAnalysis

  var rangeMethod = ""
  var errorMethod = ""
  var trackRoundoffErrs = true

  override def runPhase(ctx: Context, prg: Program): (Context, Program) = {
    rangeMethod = ctx.option[String]("rangeMethod")
    errorMethod = ctx.option[String]("errorMethod")
    trackRoundoffErrs = !ctx.hasFlag("noRoundoff")

    val choosePrecision = ctx.option[String]("choosePrecision")

    val mixedPrecision = ctx.option[Option[String]]("mixed-precision").isDefined
    val uniformPrecision = ctx.option[Precision]("precision")
    val reporter = ctx.reporter

    ctx.reporter.info(s"using $rangeMethod for ranges, $errorMethod for errors")

    var uniformPrecisions = Map[Identifier, Precision]()

    val fncsToConsider = if (ctx.hasFlag("approx")) functionsToConsider(ctx, prg).filter(_.returnType == RealType)
      else functionsToConsider(ctx, prg)

    // returns (abs error, result range, interm. errors, interm. ranges)
    val res: Map[Identifier, (Rational, Interval, Map[(Expr, PathCond), Rational], Map[(Expr, PathCond), Interval], Map[Identifier, Precision])] =
      fncsToConsider.map({ fnc =>

      val inputValMap: Map[Identifier, Interval] = ctx.specInputRanges(fnc.id)

      val fncBody = fnc.body.get

      if (choosePrecision != "no" && !ctx.specResultErrorBounds.contains(fnc.id)) {
        reporter.warning(s"Function ${fnc.id} does not have target error bound, cannot choose precision.")
      }

      if (choosePrecision != "no" && ctx.specResultErrorBounds.contains(fnc.id)) {
        reporter.info("analyzing fnc: " + fnc.id)

        // the max tolerated error
        val targetError = ctx.specResultErrorBounds(fnc.id)

        val availablePrecisions = choosePrecision match {
          case "fixed" =>
            // the max available amount of bits
            val maxBits = uniformPrecision match {
              case FixedPrecision(b) => b
              case _ => 32 // TODO put default elsewhere
            }
            (1 to maxBits).map(x => FixedPrecision(x))
          case "float" => List(Float16, Float32, Float64, DoubleDouble, QuadDouble)
          case s => throw new Exception(s"Unknown choice value for choosePrecision: $s. Stopping now")
        }

        reporter.info(s"choosing among $choosePrecision precisions")

        // save the intermediate result
        var res: (Rational, Interval, Map[(Expr, PathCond), Rational], Map[(Expr, PathCond), Interval]) = null

        // find precision which is sufficient
        availablePrecisions.find( prec => {
          try {
            reporter.debug(s"trying precision $prec")
            val allIDs = fnc.params.map(_.id)
            val inputErrorMap = allIDs.map(id => (id -> prec.absRoundoff(inputValMap(id)))).toMap
            val precisionMap: Map[Identifier, Precision] = allIDsOf(fnc.body.get).map(id => (id -> prec)).toMap

            res = computeRoundoff(inputValMap, inputErrorMap, precisionMap, fncBody,
              prec, fnc.precondition.get) // replaced ctx.specAdditionalConstraints(fnc.id)

            res._1 <= targetError
          } catch {
            case OverflowException(_) | DivisionByZeroException(_) => false // div by zero can disappear with higher precisions
          }
        }) match {

          case None =>
            val lastPrec = availablePrecisions.last
            reporter.warning(s"Highest available precision ${lastPrec} " +
              "is not sufficient. Using it anyway.")
            uniformPrecisions = uniformPrecisions + (fnc.id -> lastPrec)

          case Some(prec) =>
            uniformPrecisions = uniformPrecisions + (fnc.id -> prec)
        }
        val result: (Rational, Interval, Map[(Expr, PathCond), Rational], Map[(Expr, PathCond), Interval], Map[Identifier, Precision]) =
          (res._1, res._2, res._3, res._4, allIDsOf(fnc.body.get).map(id => (id -> uniformPrecisions(fnc.id))).toMap)
        (fnc.id -> result)

      } else {
        ctx.reporter.info("analyzing fnc: " + fnc.id)

        if (!mixedPrecision) {
          ctx.reporter.info(s"error analysis for uniform $uniformPrecision precision")
        }
        val inputErrorMap: Map[Identifier, Rational] = ctx.specInputErrors(fnc.id)

        // add variables from let statements that do not have any explicit type assignment
        val precisionMap: Map[Identifier, Precision] = ctx.specInputPrecisions(fnc.id) ++
          allIDsOf(fnc.body.get).diff(ctx.specInputPrecisions(fnc.id).keySet).map(id => (id -> uniformPrecision)).toMap
        uniformPrecisions = uniformPrecisions + (fnc.id -> uniformPrecision) // so that this info is available in codegen

        val precond = fnc.precondition.get // replaced ctx.specAdditionalConstraints(fnc.id)
        val res = computeRoundoff(inputValMap, inputErrorMap, precisionMap, fncBody,
          uniformPrecision, precond)
        val result: (Rational, Interval, Map[(Expr, PathCond), Rational], Map[(Expr, PathCond), Interval], Map[Identifier, Precision]) = (res._1, res._2, res._3, res._4, precisionMap)
        (fnc.id -> result)
      }
    }).toMap

    (ctx.copy(specInputPrecisions = ctx.specInputPrecisions ++ res.mapValues(_._5).toMap,
      uniformPrecisions = ctx.uniformPrecisions ++ uniformPrecisions,
      resultAbsoluteErrors = ctx.resultAbsoluteErrors ++ res.mapValues(_._1).toMap,
      resultRealRanges = ctx.resultRealRanges ++ res.mapValues(_._2).toMap,
      intermediateAbsErrors = ctx.intermediateAbsErrors ++ res.mapValues(_._3).toMap,
      intermediateRanges = ctx.intermediateRanges ++ res.mapValues(_._4).toMap,
      assignedPrecisions = ctx.assignedPrecisions ++ uniformPrecisions.map({case (fncid, prec) => fncid -> Map[Identifier, Precision]().withDefaultValue(prec)})
    ), prg)

  }

  def computeRange(inputValMap: Map[Identifier, Interval], expr: Expr, precond: Expr):
  (Interval, Map[(Expr, PathCond), Interval]) = {

    (rangeMethod: @unchecked) match {
      case "interval" =>
        evalRange[Interval](expr, inputValMap, Interval.apply)

      case "affine" =>
        val (rng, intrmdRange) = evalRange[AffineForm](expr,
          inputValMap.mapValues(AffineForm(_)).toMap, AffineForm.apply)
        (rng.toInterval, intrmdRange.mapValues(_.toInterval).toMap)

      case "smt" =>
        // SMT can take into account additional constraints
        val (rng, intrmdRange) = evalRange[SMTRange](expr,
          inputValMap.map({ case (id, int) => (id -> SMTRange(Variable(id), int, precond)) }),
          SMTRange.apply(_, precond))
        (rng.toInterval, intrmdRange.mapValues(_.toInterval).toMap)

      case "intervalMPFR" =>
        val (rng, intrmdRange) = evalRange[MPFRInterval](expr,
          inputValMap.mapValues(MPFRInterval(_)).toMap, MPFRInterval.apply)
        (rng.toInterval, intrmdRange.mapValues(_.toInterval).toMap)

      case "affineMPFR" =>
        val (rng, intrmdRange) = evalRange[MPFRAffineForm](expr,
          inputValMap.mapValues(MPFRAffineForm(_)).toMap, MPFRAffineForm.apply)
        (rng.toInterval, intrmdRange.mapValues(_.toInterval).toMap)
    }
  }

  def computeErrors(intermediateRanges: Map[(Expr, PathCond), Interval], inputErrorMap: Map[Identifier, Rational],
    precisionMap: Map[Identifier, Precision], expr: Expr, constPrecision: Precision):
  (Rational, Map[(Expr, PathCond), Rational]) = {

    (errorMethod: @unchecked) match {
      case "interval" =>
        val (resRoundoff, allErrors) = evalRoundoff[Interval](expr, intermediateRanges,
          precisionMap,
          inputErrorMap.mapValues(Interval.+/-).toMap,
          zeroError = Interval.zero,
          fromError = Interval.+/-,
          interval2T = Interval.apply,
          constantsPrecision = constPrecision,
          trackRoundoffErrs)

        (Interval.maxAbs(resRoundoff.toInterval), allErrors.mapValues(Interval.maxAbs).toMap)        

      case "affine" =>

        val (resRoundoff, allErrors) = evalRoundoff[AffineForm](expr, intermediateRanges,
          precisionMap,
          inputErrorMap.mapValues(AffineForm.+/-).toMap,
          zeroError = AffineForm.zero,
          fromError = AffineForm.+/-,
          interval2T = AffineForm.apply,
          constantsPrecision = constPrecision,
          trackRoundoffErrs)

        (Interval.maxAbs(resRoundoff.toInterval), allErrors.mapValues(e => Interval.maxAbs(e.toInterval)).toMap)

      case "intervalMPFR" =>

        val (resRoundoff, allErrors) = evalRoundoff[MPFRInterval](expr, intermediateRanges,
          precisionMap,
          inputErrorMap.mapValues(MPFRInterval.+/-).toMap,
          zeroError = MPFRInterval.zero,
          fromError = MPFRInterval.+/-,
          interval2T = MPFRInterval.apply,
          constantsPrecision = constPrecision,
          trackRoundoffErrs)

        (Interval.maxAbs(resRoundoff.toInterval), allErrors.mapValues(e => Interval.maxAbs(e.toInterval)).toMap)

      case "affineMPFR" =>

        val (resRoundoff, allErrors) = evalRoundoff[MPFRAffineForm](expr, intermediateRanges,
          precisionMap,
          inputErrorMap.mapValues(MPFRAffineForm.+/-).toMap,
          zeroError = MPFRAffineForm.zero,
          fromError = MPFRAffineForm.+/-,
          interval2T = MPFRAffineForm.apply,
          constantsPrecision = constPrecision,
          trackRoundoffErrs)

        (Interval.maxAbs(resRoundoff.toInterval), allErrors.mapValues(e => Interval.maxAbs(e.toInterval)).toMap)
    }
  }

  def computeRoundoff(inputValMap: Map[Identifier, Interval], inputErrorMap: Map[Identifier, Rational],
    precisionMap: Map[Identifier, Precision], expr: Expr, constPrecision: Precision, precond: Expr):
    (Rational, Interval, Map[(Expr, PathCond), Rational], Map[(Expr, PathCond), Interval]) = {

    try{
      val (resRange, intermediateRanges) = computeRange(inputValMap, expr, precond)

      val (resError, intermediateErrors) = computeErrors(intermediateRanges, inputErrorMap, precisionMap, expr,
        constPrecision)
    
      (resError, resRange, intermediateErrors, intermediateRanges)
    }
    catch {
      case e: OverflowFixException => {
        println("OverflowFix in interval roundoff")
        println(e.identifiers)
        // in this case, we will increase the precision of identifier by 1 in precisionMap
        // iterate over all identifiers and increase precision by 1
        var newPrecisionMap: Map[Identifier, Precision] = precisionMap
        for (identifier <- e.identifiers) {
          val currPrecision = precisionMap(identifier)
          val incPrecision = currPrecision match {
            case FixedPrecision(16) => FixedPrecision(32)
            case FixedPrecision(32) => FixedPrecision(32) // TODO: this is wrong, but I believe it will fix the problem for now
            case _ => {
              throw new Exception("OverflowFix in interval roundoff, cannot increase precision: " + currPrecision)
            }
          }
          println("Overflow, increasing precision of " + identifier + " to " + incPrecision)
          newPrecisionMap = newPrecisionMap.map({case (id, prec) => if (id == identifier) (id -> incPrecision) else (id -> prec)})
        }

        computeRoundoff(inputValMap, inputErrorMap, newPrecisionMap, expr, constPrecision, precond)
      }
      case e: OverflowException => {
        println("Overflow in interval roundoff")
        throw new Exception("Overflow in interval roundoff")
      }
      case e: Exception => {
        println("Exception in interval roundoff")
        throw new Exception("Exception in interval roundoff")
      }
    }
  }
}
