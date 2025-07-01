// Copyright 2017 MPI-SWS, Saarbruecken, Germany

package daisy
package analysis

import daisy.lang.Identifiers._
import daisy.lang.TreeOps.containsApproxNode
import daisy.lang.Trees._
import daisy.tools.FinitePrecision._
import daisy.tools.{AffineForm, Interval, Rational}

/**
  * This phase computes roundoff errors for a given precision and data type
  * and attaches the information to the trees.
  * *
  * Note:
  *- if we want to use separation of errors, then the range computation needs
  * to compute the actual (but real-valued) ranges, as
  * |f(\tl{x} - \tl{f}(\tl{x}) | is over \tl{x} and hence the actual range.
  *- For now, we use affine arithmetic to track initial as well as roundoff
  *errors. Via options one can switch off tracking of either.
  *- if we are tracking initial errors, and one is present, do NOT add an
  * additional roundoff error
  * *
  * Prerequisites:
  *- SpecsProcessingPhase
  *- RangePhase (check you are computing the correct ranges)
  */
object AbsErrorPhase extends DaisyPhase with tools.RoundoffEvaluators {
  override val name = "Roundoff"
  override val description = "Computes roundoff errors"

  override val definedOptions: Set[CmdLineOption[Any]] = Set(
    StringChoiceOption(
      "errorMethod",
      Set("affine", "interval", "intervalMPFR", "affineMPFR"),
      "affine",
      "Method for error analysis")
  )

  override implicit val debugSection = DebugSectionAnalysis

  var errorMethod = ""

  override def runPhase(ctx: Context, prg: Program): (Context, Program) = {
    errorMethod = ctx.option[String]("errorMethod")

    val trackRoundoffErrs = !ctx.hasFlag("noRoundoff")
    val uniformPrecision = ctx.option[Precision]("precision")

    val withApproxNodes = functionsToConsider(ctx, prg).filter(fnc => containsApproxNode(fnc.body.get))
    val noApproxNodes = functionsToConsider(ctx, prg).diff(withApproxNodes)

    val resNoApprox: Map[Identifier, (Rational, Map[(Expr, PathCond), Rational])] =
      noApproxNodes.map(fnc => computeRoundoffError(fnc, uniformPrecision, trackRoundoffErrs, ctx, recompute = !ctx.hasFlag("approx") || ctx.recomputeAbsErrors)).toMap

    val updatedContext = ctx.copy(resultAbsoluteErrors = ctx.resultAbsoluteErrors ++ resNoApprox.mapValues(_._1))
    val resWithApprox: Map[Identifier, (Rational, Map[(Expr, PathCond), Rational])] =
      withApproxNodes.map(fnc => computeRoundoffError(fnc, uniformPrecision, trackRoundoffErrs, updatedContext, recompute = true)).toMap

    val res = resNoApprox ++ resWithApprox
    (ctx.copy(
      resultAbsoluteErrors = ctx.resultAbsoluteErrors ++ res.mapValues(_._1).toMap,
      intermediateAbsErrors = ctx.intermediateAbsErrors ++ res.mapValues(_._2).toMap),
      prg)
  }

  private def computeRoundoffError(fnc: FunDef, uniformPrecision: Precision, trackRoundoffErrs: Boolean, ctx: Context, recompute: Boolean): (Identifier, (Rational, Map[(Expr, PathCond), Rational])) = {

    if (!recompute)
      fnc.id -> (ctx.resultAbsoluteErrors(fnc.id), ctx.intermediateAbsErrors(fnc.id))
    else {

      val fncBody = fnc.body.get
      val intermediateRanges = ctx.intermediateRanges(fnc.id)
      val precisionMap: Map[Identifier, Precision] = ctx.specInputPrecisions(fnc.id)
      // needs to be re-computed because e.g. mixed-tuning may have assigned different precision
      val inputErrorMap: Map[Identifier, Rational] = lang.TreeOps.freeVariablesOf(fncBody).map({
        case id: Identifier =>
          (id -> precisionMap(id).absRoundoff(intermediateRanges((Variable(id), emptyPath))))
      }).toMap

      errorMethod match {
        case "interval" =>
          val (resRoundoff, allErrors) = evalRoundoff[Interval](fncBody, intermediateRanges,
            precisionMap,
            inputErrorMap.mapValues(Interval.+/-).toMap,
            zeroError = Interval.zero,
            fromError = Interval.+/-,
            interval2T = Interval.apply,
            constantsPrecision = uniformPrecision,
            trackRoundoffErrs,
            resultAbsErrors = ctx.resultAbsoluteErrors,
            resultErrorsMetalibm = ctx.approxReportedErrors.getOrElse(fnc.id, Map()) // if the function doesn't contain Approx nodes the map will be empty
          )

          return fnc.id -> (Interval.maxAbs(resRoundoff), allErrors.mapValues(aa => Interval.maxAbs(aa)).toMap)
        case "affine" =>{
          val (resRoundoff, allErrors) = evalRoundoff[AffineForm](fncBody, intermediateRanges,
            precisionMap,
            inputErrorMap.mapValues(AffineForm.+/-).toMap,
            zeroError = AffineForm.zero,
            fromError = AffineForm.+/-,
            interval2T = AffineForm.apply,
            constantsPrecision = uniformPrecision,
            trackRoundoffErrs,
            resultAbsErrors = ctx.resultAbsoluteErrors,
            resultErrorsMetalibm = ctx.approxReportedErrors.getOrElse(fnc.id, Map()) // if the function doesn't contain Approx nodes the map will be empty
          )

          return fnc.id -> (Interval.maxAbs(resRoundoff.toInterval), allErrors.mapValues(aa => Interval.maxAbs(aa.toInterval)).toMap)
        }
      }
    }
  }

}
