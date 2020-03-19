#include <cmath>
#include <aw_kogmo_math.h>
#include <poly_traj.h>
#include <trajectory_points_interface.h>

#include "curveSmoother.h"

namespace vlr {

const uint32_t CurveSmoother::cs_temp_buf_size_;

CurveSmoother::CurveSmoother() {
  aux_ = new double[cs_temp_buf_size_]; // solve_system()
  coeffa_ = new double[cs_temp_buf_size_]; // from deboor()


  xp_ = new double[cs_temp_buf_size_];
  yp_ = new double[cs_temp_buf_size_];
  alpha_ = new double[cs_temp_buf_size_];
  beta_ = new double[cs_temp_buf_size_];
  gamma_ = new double[cs_temp_buf_size_];
  up_ = new double[cs_temp_buf_size_];
  low_ = new double[cs_temp_buf_size_];
  ergx_ = new double[cs_temp_buf_size_];
  ergy_ = new double[cs_temp_buf_size_];
  curvatures_ = new double[cs_temp_buf_size_];
  bez_x_ = new double[cs_temp_buf_size_];
  bez_y_ = new double[cs_temp_buf_size_];
  lookahead_x_ = new double[cs_temp_buf_size_];
  lookahead_y_ = new double[cs_temp_buf_size_];

  cp_.kappa_prime = 0;
}

CurveSmoother::~CurveSmoother() {
  delete[] aux_;
  delete[] coeffa_;
  delete[] xp_;
  delete[] yp_;
  delete[] alpha_;
  delete[] beta_;
  delete[] gamma_;
  delete[] up_;
  delete[] low_;
  delete[] ergx_;
  delete[] ergy_;
  delete[] curvatures_;
  delete[] bez_x_;
  delete[] bez_y_;

}

//bool SmoothCurvePoints(CurvePoints& incurve,
//    CurvePoints& outcurve);

// Finds the C2 cubic spline interpolant to the data points in data_x, data_y.
// Input:
// knot:  the knot sequence  knot[0], ..., knot[l]
// l: the number of intervals
// data_x, data_y: the data points data_x[0], ..., data[l+2].
// Attention:
// data_x[1] and data_x[l+1] are filled by  Bessel end conditions and are
// thus ignored on input. Same for data_y.
// Output:
// bspl_x, bspl_y: the B-spline control polygon of the interpolant. Dimensions:
// bspl_x[0], ..., bspl_x[l+2]. Same for bspl_y.

void CurveSmoother::sample_as_cbspline_interp(int degree, int dense,
                dgc::PlannerTrajectory& plan, std::vector<CurvePoint>* splinepoints) {
  int l;
  uint32_t num_points, numResPoints;
  double* knots = NULL, *tx = NULL, *ty = NULL;

  num_points = std::min((uint32_t)((plan.num_waypoints >> 1) << 1), cs_temp_buf_size_-1);

  if (num_points < 4) {
    printf("Not enough points (min 4)!\n");
    return;
  }

  splinepoints->clear();

//  memcpy(&xp_[1], xcoord, numPoints * sizeof(double));
//  memcpy(&yp_[1], ycoord, numPoints * sizeof(double));

  for (uint32_t i = 0; i < num_points; i++) {
    xp_[i] = plan.waypoint[i].x;
    yp_[i] = plan.waypoint[i].y;
//    xp_[i+1] = plan.waypoint[i].x;
//    yp_[i+1] = plan.waypoint[i].y;
  }

//  xp_[0] = xp_[1];
//  yp_[0] = yp_[1];
//  xp_[numPoints + 1] = xp_[numPoints];
//  yp_[numPoints + 1] = yp_[numPoints];

  l = num_points - 1;
//  l = num_points - degree + 2;

  if (!(knots = create_centripetal_knotsequence(xp_, yp_, l, degree))) {
    printf("Fehler beim Erstellen der Knotesequenz.");
    return;
  }

  // tridiagonales Gleichungssytem aufstellen
  set_up_system(&knots[degree - 1], l, alpha_, beta_, gamma_);

  // LU-Zerlegung
  l_u_system(alpha_, beta_, gamma_, l, up_, low_);

  // Kurvenenden korrigieren
  bessel_ends(xp_, &knots[degree - 1], l);
  bessel_ends(yp_, &knots[degree - 1], l);

  // Gleichungssytem loesen

  if (!(tx = solve_system(up_, low_, gamma_, l, xp_))) {
    printf("Fehler beim Loesen des Gleichungssystems fuer X-Koordinaten.");
    free(knots);
    return;
  }

  if (!(ty = solve_system(up_, low_, gamma_, l, yp_))) {
    printf("Fehler beim Loesen des Gleichungssystems fuer Y-Koordinaten.");
    free(knots);
    free(tx);
    return;
  }

  bspline2points(degree, l, tx, knots, dense, ergx_, (int*)&numResPoints);
  bspline2points(degree, l, ty, knots, dense, ergy_, (int*)&numResPoints);

  bspl_kappas(tx, ty, knots, l, dense, curvatures_);
//  bspl_kappas(ergx_, ergy_, knots, l, dense, curvatures_);

  splinepoints->resize(numResPoints);

  static double s = 0;
  for (uint32_t i = 0; i < numResPoints; i++) {
    if(i==0) {(*splinepoints)[i].s = 0;}
     else {
       s+=hypot(ergx_[i]-ergx_[i-1], ergy_[i]-ergy_[i-1]);
       (*splinepoints)[i].s = s;
    }
    (*splinepoints)[i].x = ergx_[i];
    (*splinepoints)[i].y = ergy_[i];
    if (i == 0 || (ergy_[i] - ergy_[i - 1] <0.0000001 && ergx_[i] - ergx_[i - 1] <0.0000001 && i !=num_points-1)) {
      (*splinepoints)[i].theta = atan2(ergy_[i+1] - ergy_[i], ergx_[i+1] - ergx_[i]);
    }
    else {
//      (*splinepoints)[i].theta = 0;
      (*splinepoints)[i].theta = atan2(ergy_[i] - ergy_[i - 1], ergx_[i] - ergx_[i - 1]);
    }
    (*splinepoints)[i].kappa = curvatures_[i];
//    printf("%i -- %f, %f %f %f \n", i, (*splinepoints)[i].x, (*splinepoints)[i].y, (*splinepoints)[i].theta, (*splinepoints)[i].kappa);
  }

  free(knots);
  free(tx);
  free(ty);
}

void CurveSmoother::sample_as_cbspline(int degree, int dense,
                dgc::PlannerTrajectory& plan, std::vector<CurvePoint>* splinepoints) {
  int l;
  uint32_t num_points, numResPoints;
  double* knots = NULL;

  num_points = std::min((uint32_t)((plan.num_waypoints >> 1) << 1), cs_temp_buf_size_-1);

  if (num_points < 4) {
    printf("Not enough points (min 4)!\n");
    return;
  }

  splinepoints->clear();

//  memcpy(&xp_[1], xcoord, numPoints * sizeof(double));
//  memcpy(&yp_[1], ycoord, numPoints * sizeof(double));

  for (uint32_t i = 0; i < num_points; i++) {
    xp_[i] = plan.waypoint[i].x;
    yp_[i] = plan.waypoint[i].y;
//    xp_[i+1] = plan.waypoint[i].x;
//    yp_[i+1] = plan.waypoint[i].y;
  }

//  xp_[0] = xp_[1];
//  yp_[0] = yp_[1];
//  xp_[numPoints + 1] = xp_[numPoints];
//  yp_[numPoints + 1] = yp_[numPoints];

  l = num_points - 1;//degree;// + 2;

  if (!(knots = create_centripetal_knotsequence(xp_, yp_, l, degree))) {
    printf("Fehler beim Erstellen der Knotesequenz.");
    return;
  }

  // Kurvenenden korrigieren
  bessel_ends(xp_, &knots[degree - 1], l);
  bessel_ends(yp_, &knots[degree - 1], l);

  bspline2points(degree, l, xp_, knots, dense, ergx_, (int*)&numResPoints);
  bspline2points(degree, l, yp_, knots, dense, ergy_, (int*)&numResPoints);

  bspl_kappas(ergx_, ergy_, knots, l, dense, curvatures_);

  splinepoints->resize(numResPoints);

  static double s = 0;
  for (uint32_t i = 0; i < numResPoints; i++) {
    if(i==0) {(*splinepoints)[i].s = 0;}
     else {
       s+=hypot(ergx_[i]-ergx_[i-1], ergy_[i]-ergy_[i-1]);
       (*splinepoints)[i].s = s;
    }
    (*splinepoints)[i].x = ergx_[i];
    (*splinepoints)[i].y = ergy_[i];
    if (i == 0 || (ergy_[i] - ergy_[i - 1] <0.0000001 && ergx_[i] - ergx_[i - 1] <0.0000001 && i !=num_points-1)) {
      (*splinepoints)[i].theta = atan2(ergy_[i+1] - ergy_[i], ergx_[i+1] - ergx_[i]);
    }
    else {
//      (*splinepoints)[i].theta = 0;
      (*splinepoints)[i].theta = atan2(ergy_[i] - ergy_[i - 1], ergx_[i] - ergx_[i - 1]);
    }
    (*splinepoints)[i].kappa = 0;//curvatures_[i];
  }

  free(knots);
}

void CurveSmoother::sample_as_bezier(int degree, int dense,
    std::vector<CurvePoint>& points, std::vector<CurvePoint>* splinepoints) {
  int l;
  uint32_t num_points, numResPoints;
  double* knots = NULL;

  num_points = std::min((uint32_t)((points.size() >> 1) << 1), cs_temp_buf_size_-1);

    // TODO: 4 is only valid for cubic curves...
  if (num_points < 4) {
    printf("Not enough points (min 4)!\n");
    return;
  }

    // number of intervals
  l = (int)num_points-1;
  splinepoints->clear();

//  memcpy(&xp_[1], xcoord, numPoints * sizeof(double));
//  memcpy(&yp_[1], ycoord, numPoints * sizeof(double));

  for (uint32_t i = 0; i < num_points; i++) {
    xp_[i] = points[i].x;
    yp_[i] = points[i].y;
//    xp_[i+1] = plan.waypoint[i].x;
//    yp_[i+1] = plan.waypoint[i].y;
  }
  printf("%i input points.\n", num_points);

//  xp_[0] = xp_[1];
//  yp_[0] = yp_[1];
//  xp_[numPoints + 1] = xp_[numPoints];
//  yp_[numPoints + 1] = yp_[numPoints];

//  bezier2points(degree, l, xp_, dense, ergx_, (int*)&numResPoints);
//  bezier2points(degree, l, yp_, knots, dense, ergy_, (int*)&numResPoints);

    l = (num_points - 1) / 3;

    bezier2points(degree, l, xp_, dense, ergx_, (int*)&numResPoints);
    bezier2points(degree, l, yp_, dense, ergy_, (int*)&numResPoints);


    bezier_kappas(xp_, yp_, degree, l, dense, curvatures_);
//  bspl_kappas(ergx_, ergy_, knots, l, dense, curvatures_);

  splinepoints->resize(numResPoints);

  static double s = 0;
  for (uint32_t i = 0; i < numResPoints; i++) {
    if(i==0) {(*splinepoints)[i].s = 0;}
     else {
       s+=hypot(ergx_[i]-ergx_[i-1], ergy_[i]-ergy_[i-1]);
       (*splinepoints)[i].s = s;
    }
    (*splinepoints)[i].x = ergx_[i];
    (*splinepoints)[i].y = ergy_[i];
    if (i == 0 || (ergy_[i] - ergy_[i - 1] <0.0000001 && ergx_[i] - ergx_[i - 1] <0.0000001 && i !=num_points-1)) {
      (*splinepoints)[i].theta = atan2(ergy_[i+1] - ergy_[i], ergx_[i+1] - ergx_[i]);
    }
    else {
//      (*splinepoints)[i].theta = 0;
      (*splinepoints)[i].theta = atan2(ergy_[i] - ergy_[i - 1], ergx_[i] - ergx_[i - 1]);
    }
    (*splinepoints)[i].kappa = curvatures_[i];
  }

  free(knots);
}

//void CurveSmoother::sample_as_cbspline(int degree, int dense, double* xcoord, double* ycoord, int numPoints,
//    CurvePoints& splinepoints) {
//  int numResPoints, l, i;
//  double ergx_[C2_CURVEPOINTS_POINTSMAX], ergy_[C2_CURVEPOINTS_POINTSMAX];
//
//  if (numPoints < 4) {
//    return;
//  }
//
//  l = (numPoints - 1) / 3;
//
//  bezier2points(degree, l, xcoord, dense, ergx_, &numResPoints);
//  bezier2points(degree, l, ycoord, dense, ergy_, &numResPoints);
//  splinepoints.numberValidPoints = numResPoints;
//
//  //    for(i=0; i<numPoints; i++) printf("(%f, %f)\n", xcoord[i], ycoord[i]);
//
//  //l=numPoints-degree;
//
//  //if(!(knots=create_centripetal_knotsequence(xcoord, ycoord, l, degree)))
//  //  {
//  //  printf("Fehler beim Erstellen der Knotesequenz.");
//  //  return;
//  //  }
//
//  //bspl_to_points(degree, l, xcoord, knots, dense, ergx_, &numResPoints);
//  //bspl_to_points(degree, l, ycoord, knots, dense, ergy_, &numResPoints);
//
//  //  printf("lip: (%f, %f), vlip: (%f, %f)\n", xcoord[numPoints-2], ycoord[numPoints-2], xcoord[numPoints-1], ycoord[numPoints-1]);
//  std::cout << std::flush;
//  for (i = 0; i < numResPoints; i++) {
//    splinepoints[i].x = ergx_[i];
//    splinepoints[i].y = ergy_[i];
//    if (i < numResPoints - 1) {
//      splinepoints[i].theta = atan2(ergy_[i + 1] - ergy_[i], ergx_[i + 1] - ergx_[i]);
//      //    printf("(%f, %f)\n", ergx_[i], ergy_[i]);
//      //    printf("theta: %f\n", splinepoints[i].theta);
//    }
//    else {
//      splinepoints[i].theta = atan2(ergy_[i] - ergy_[i - 1], ergx_[i] - ergx_[i - 1]);
//    }
//
//    //    if (i>1 && i<numResPoints-1)
//    {
//      splinepoints[i].kappa = bezCurvature0(&ergx_[i], &ergy_[i], degree);
//      if (STRAIGHT_CURVATURE(splinepoints[i].kappa)) {
//        splinepoints[i].kappa = 0.0;
//      }
//      if (splinepoints[i].kappa > CURVATURE_THRESH || splinepoints[i].kappa < -CURVATURE_THRESH) {
//        splinepoints[i].kappa = splinepoints[i - 1].kappa;
//      }
//      //    double delta1=std::sqrt((ergx_[i]-ergx_[i-1])*(ergx_[i]-ergx_[i-1])+(ergy_[i]-ergy_[i-1])*(ergy_[i]-ergy_[i-1]));
//      //      double delta2=std::sqrt((ergx_[i]-ergx_[i+1])*(ergx_[i]-ergx_[i+1])+(ergy_[i]-ergy_[i+1])*(ergy_[i]-ergy_[i+1]));
//      //      double deltadiff=delta2-delta1, tmp=0.25*(delta2+delta1)*(delta2+delta1)+1.0;
//      //      if(!FZERO(deltadiff))
//      //          {
//      //          splinepoints[i].kappa=-deltadiff/sqrt(tmp*tmp*tmp);
//      ////            splinepoints[i].kappa=std::fabs(deltadiff)/sqrt(tmp*tmp*tmp);
//      //          if(fabs(splinepoints[i].kappa)>CURVATURE_THRESH) {splinepoints[i].kappa=splinepoints[i-1].kappa;}
//      //          }
//      //      else
//      //          {splinepoints[i].kappa=0;}
//    }
//    //          else
//    //          {
//    //              splinepoints[i].kappa=0;
//    //          }
//  }
//}

/*
 -------------------------------------------------------------------------------------------
 Funktion : Zentripedale 2D-Parametrisierung (Knotensequenz) Achtung: not (knot[l]=1.0)
 data_x/y[1], data_x/y[l+1] werden nicht benutzt
 Parameter  : data_x, data_y - Koordinaten der Punkte zu denen die Knotensequenz erstellt
 werden soll (0 ... l+2)
 l         - Anzahl der Intervalle
 Rckgabe  : Erzeugte Knotensequenz oder NULL (Fehler)
 -------------------------------------------------------------------------------------------
 */
double* CurveSmoother::create_centripetal_knotsequence(double* data_x, double* data_y, int l, int degree) {
  double* knots = NULL;
  int i, l1, l2; // In the following, special care must be
  double delta; // at the ends because of the data structure
  // used. See note above.

  l1 = l - 1;
  l2 = l + 2;

  if (!(knots = (double*) malloc((l + 2 * degree - 1) * sizeof(double)))) {
    return (NULL);
  }

  // Erste Knoten bei Parameter 0
  for (i = 0; i < degree; i++) {
    knots[i] = 0.0;
  }

  delta = sqrt((data_x[2] - data_x[0]) * (data_x[2] - data_x[0]) + (data_y[2] - data_y[0]) * (data_y[2] - data_y[0]));

  knots[degree] = sqrt(delta);

  for (i = 2; i < l; i++) {
    delta = sqrt((data_x[i + 1] - data_x[i]) * (data_x[i + 1] - data_x[i]) + (data_y[i + 1] - data_y[i]) * (data_y[i
        + 1] - data_y[i]));
    knots[i + degree - 1] = knots[i + degree - 2] + sqrt(delta);
  }

  delta = sqrt((data_x[l2] - data_x[l]) * (data_x[l2] - data_x[l]) + (data_y[l2] - data_y[l])
      * (data_y[l2] - data_y[l]));
  knots[l + degree - 1] = knots[l + degree - 2] + sqrt(delta);

  // Knotensequenz normieren
  for (i = degree; i < l + degree; i++) {
    knots[i] /= knots[l + degree - 1];
  };
  for (i = l + degree; i < l + 2 * degree - 1; i++) {
    knots[i] = 1.0;
  }
  return (knots);
}

/*
 -------------------------------------------------------------------------------------------
 Funktion : Lineares Gleichungssytem fr B-Spline-Interpolation aus der Knotensequenz
 erzeugen ("clamped ends")
 Parameter  : knots        - Knotensequenz
 l           - Anzahl der Intervalle
 alpha, beta, gamma - 1D-Felder, die die Interpolationsmatrix festlegen
 Rckgabe  : -
 -------------------------------------------------------------------------------------------
 */
void CurveSmoother::set_up_system(double* knots, int l, double* alpha, double* beta, double* gamma) {
  int i, l1;
  double delta_im2, delta_im1, delta_i, delta_ip1, sum;
  l1 = l - 1;

  // some special cases:
  if (l == 1) {
    alpha[0] = 0.0;
    alpha[1] = 0.0;
    beta[0] = 1.0;
    beta[1] = 1.0;
    gamma[0] = 0.0;
    gamma[1] = 0.0;
    return;
  }

  if (l == 2) {
    beta[0] = 1.0;
    delta_im1 = (knots[1] - knots[0]);
    delta_i = (knots[2] - knots[1]);
    delta_ip1 = (knots[3] - knots[2]);
    sum = delta_im1 + delta_i;

    alpha[1] = delta_i * delta_i / sum;
    beta[1] = (delta_i * delta_im1) / sum + delta_im1 * delta_i / sum;
    gamma[1] = delta_im1 * delta_im1 / sum;

    alpha[1] = alpha[1] / sum;
    beta[1] = beta[1] / sum;
    gamma[1] = gamma[1] / sum;

    beta[2] = 1.0;
    alpha[2] = 0.0;
    gamma[2] = 0.0;
    return;
  }

  // the rest does the cases l>2.
  delta_im1 = (knots[1] - knots[0]);
  delta_i = (knots[2] - knots[1]);
  delta_ip1 = (knots[3] - knots[2]);
  sum = delta_im1 + delta_i;

  beta[0] = 1.0;
  gamma[0] = 0.0;

  alpha[1] = delta_i * delta_i / sum;
  beta[1] = (delta_i * delta_im1) / sum + delta_im1 * (delta_i + delta_ip1) / (sum + delta_ip1);
  gamma[1] = delta_im1 * delta_im1 / (sum + delta_ip1);

  alpha[1] = alpha[1] / sum;
  beta[1] = beta[1] / sum;
  gamma[1] = gamma[1] / sum;

  // Now for the main loop:
  for (i = 2; i < l1; i++) {
    // compute delta_i_minus_2,...
    delta_im2 = (knots[i - 1] - knots[i - 2]);
    delta_im1 = (knots[i] - knots[i - 1]);
    delta_i = (knots[i + 1] - knots[i]);
    delta_ip1 = (knots[i + 2] - knots[i + 1]);

    sum = delta_im1 + delta_i;

    alpha[i] = delta_i * delta_i / (delta_im2 + sum);
    beta[i] = delta_i * (delta_im2 + delta_im1) / (delta_im2 + sum) + delta_im1 * (delta_i + delta_ip1) / (sum
        + delta_ip1);
    gamma[i] = delta_im1 * delta_im1 / (sum + delta_ip1);

    alpha[i] = alpha[i] / sum;
    beta[i] = beta[i] / sum;
    gamma[i] = gamma[i] / sum;
  }

  //  special care at the end:
  delta_im2 = knots[l - 2] - knots[l - 3];
  delta_im1 = knots[l1] - knots[l - 2];
  delta_i = knots[l] - knots[l1];
  sum = delta_im1 + delta_i;

  alpha[l1] = delta_i * delta_i / (delta_im2 + sum);
  beta[l1] = delta_i * (delta_im2 + delta_im1) / (delta_im2 + sum) + delta_im1 * delta_i / sum;
  gamma[l1] = delta_im1 * delta_im1 / sum;

  alpha[l1] = alpha[l1] / sum;
  beta[l1] = beta[l1] / sum;
  gamma[l1] = gamma[l1] / sum;

  alpha[l] = 0.0;
  beta[l] = 1.0;
  gamma[l] = 0.0;
}

/*
 -------------------------------------------------------------------------------------------
 Funktion : Berechnen der LU-Zerlegung eines tridiagonalen, linearen Gleichungssystems
 Parameter  : alpha, beta, gamma - untere, mittlere und obere Diagonalenelemente
 l           - Gre der Matrix [0,l]x[0,l]
 low, up       - untere und obere Dreiecksmatrix (Ergebnis)
 Rckgabe  : -
 -------------------------------------------------------------------------------------------
 */
void CurveSmoother::l_u_system(double* alpha, double* beta, double* gamma, int l, double* up, double* low) {
  int i;

  up[0] = beta[0];

  for (i = 1; i <= l; i++) {
    low[i] = alpha[i] / up[i - 1];
    up[i] = beta[i] - low[i] * gamma[i - 1];
  }
}

/*
 -------------------------------------------------------------------------------------------
 Funktion : Berechnen der B-Spline-Endpunkte data[1] und data[l+1] nach der
 Bessel-Bedingung
 Parameter  : data  - Datenpunkte, deren Enden angepat werden sollen (data[1] und
 data[l+1] werden eingefgt => Ergebnis)
 knots  - Knotensequenz
 l    - Anzahl der Intervalle
 Rckgabe  : -
 -------------------------------------------------------------------------------------------
 */
void CurveSmoother::bessel_ends(double* data, double* knots, int l) {
  double alpha, beta;

  if (l == 1) {
    /* This is not really Bessel, but then what do you do
     when you have only one interval? -- make it linear! */
    data[1] = (2.0 * data[0] + data[3]) / 3.0;
    data[2] = (2.0 * data[3] + data[0]) / 3.0;
  }

  else if (l == 2) {
    // beginning:
    alpha = (knots[2] - knots[1]) / (knots[2] - knots[0]);
    beta = 1.0 - alpha;

    data[1] = (data[2] - alpha * alpha * data[0] - beta * beta * data[4]) / (2.0 * alpha * beta);
    data[1] = 2.0 * (alpha * data[0] + beta * data[1]) / 3.0 + data[0] / 3.0;

    // end:
    alpha = (knots[2] - knots[1]) / (knots[2] - knots[0]);
    beta = 1.0 - alpha;

    data[3] = (data[2] - alpha * alpha * data[0] - beta * beta * data[4]) / (2.0 * alpha * beta);

    data[3] = 2.0 * (alpha * data[3] + beta * data[4]) / 3.0 + data[4] / 3.0;
  }

  else

  {
    // beginning:
    alpha = (knots[2] - knots[1]) / (knots[2] - knots[0]);
    beta = 1.0 - alpha;

    data[1] = (data[2] - alpha * alpha * data[0] - beta * beta * data[3]) / (2.0 * alpha * beta);
    data[1] = 2.0 * (alpha * data[0] + beta * data[1]) / 3.0 + data[0] / 3.0;

    // end:
    alpha = (knots[l] - knots[l - 1]) / (knots[l] - knots[l - 2]);
    beta = 1.0 - alpha;

    data[l + 1] = (data[l] - alpha * alpha * data[l - 1] - beta * beta * data[l + 2]) / (2.0 * alpha * beta);

    data[l + 1] = 2.0 * (alpha * data[l + 1] + beta * data[l + 2]) / 3.0 + data[l + 2] / 3.0;
  }
}

/*
 -------------------------------------------------------------------------------------------
 Funktion : Lsen eines tridiagonalen, linearen Gleichungssystems der Gre (l+1)(l+1).
 Parameter  : up, down - Teilmatrizen der LU-Zerlegung
 gamma     - Obere Diagonale der Orginalmatrix
 rhs     - l+2 Datenpunkte mit den indirekten Bezier-Tangentenvektoren (right hand
 side der Matrix) an den Enden (rhs[1] und rhs[l+1])
 Rckgabe  : Lsungsvektor mit l+2 Elementen oder NULL (Fehler)
 -------------------------------------------------------------------------------------------
 */
double* CurveSmoother::solve_system(double* up, double* low, double* gamma, int l, double* rhs) {
  int i;
  double* d = NULL;

  if (!(d = (double*) malloc((l + 3) * sizeof(double)))) {
    return (NULL);
  }

  d[0] = rhs[0];
  d[1] = rhs[1];

  // forward substitution:
  aux_[0] = rhs[1];
  for (i = 1; i <= l; i++) {
    aux_[i] = rhs[i + 1] - low[i] * aux_[i - 1];
  }

  // backward substitution:
  d[l + 1] = aux_[l] / up[l];
  for (i = l - 1; i > 0; i--) {
    d[i + 1] = (aux_[i] - gamma[i] * d[i + 2]) / up[i];
  }

  d[l + 2] = rhs[l + 2];
  return (d);
}

// generates points on B-spline curve. (one coordinate)
// input: degree: polynomial degree of each piece of curve
//  l:  number of intervals
//  coeff:  B-spline control points
//  knot: knot sequence: knot[0]...knot[l+2*degree-2]
//  dense:  how many points per segment
// output:  points: output array.
//  point_num: how many points are generated.

void CurveSmoother::bspline2points(int degree, int l, double* coeff, double* knots, int dense, double* points,
    int* point_num) {
  int i, ii, l2;
  double u;

  l2 = l + degree - 1;

  *point_num = 0;
  for (i = +degree - 1; i < l + degree - 1; i++) {
    if (knots[i + 1] > knots[i])
      for (ii = 0; ii <= dense; ii++) {
      u = knots[i] + ii * (knots[i + 1] - knots[i]) / dense;
      points[*point_num] = deboor(degree, coeff, knots, u, i);

      *point_num = (*point_num) + 1;
    }
  }
}

// uses de Boor algorithm to compute one
// coordinate on B-spline curve for param. value u in interval i.
// input:  degree:  polynomial degree of each piece of curve
// coeff: B-spline control points
// knot:  knot sequence
// u: evaluation abscissa
// i: u's interval: u[i]<= u < u[i+1]
// output: coordinate value.
double CurveSmoother::deboor(int degree, double* coeff, double* knot, double u, int i) {
  int k, j;
  double t1, t2;

  for (j = i - degree + 1; j <= i + 1; j++) {
    coeffa_[j] = coeff[j];
  }

  for (k = 1; k <= degree; k++) {
    for (j = i + 1; j >= i - degree + k + 1; j--) {
      t1 = (knot[j + degree - k] - u) / (knot[j + degree - k] - knot[j - 1]);
      t2 = 1.0 - t1;

      coeffa_[j] = t1 * coeffa_[j - 1] + t2 * coeffa_[j];
    }
  }
  return (coeffa_[i + 1]);
}

/*  curvatures of cubic B-spline curve into a file.
input:
  bspl_x,bspl_y:  2D rat. B-spline polygon
  knot:   the knot sequence
  dense:  how many curvature values to compute per interval
  l:      no. of intervals
*/
void CurveSmoother::bspl_kappas(double* bspl_x, double* bspl_y, double* knot, int l, int dense,
                                double* curvatures) {

  double bleftx[4],blefty[4],brightx[4],brighty[4];
  double coeffx[4],coeffy[4];
  double t,delt,h,u,diff;
  int i,j,i3, k=0;
//  double curvature_0(),curvature_1(),abs();

  /* first, convert B-spline to Bezier: */

//  ratbspline_to_bezier(bspl_x,bspl_y,bspl_w,knot,l, bez_x_,bez_y_,bez_w);
  bspline_to_bezier(bspl_x, knot, l, bez_x_);
  bspline_to_bezier(bspl_y, knot, l, bez_y_);

  /* Now plot kappas for each interval. */

//  fprintf(outfile,"%d  %d\n",dense*l,dense);
  for (i = 0; i < l; i++) {
    i3 = i * 3;
    u = knot[i]; /*starting value for i-th segment*/
    diff = knot[i + 1] - knot[i];
    for (j = 0; j <= 3; j++) { /* create i-th cubic Bezier curve: */
      coeffx[j] = bez_x_[i3 + j];
      coeffy[j] = bez_y_[i3 + j];
    }

    delt = 1.0 / (double) dense;

    for (t = 0.0; t < 0.5; t = t + delt) {
      subdiv(3, coeffx, t, bleftx, brightx);
      subdiv(3, coeffy, t, blefty, brighty);
      h = curvature_0(brightx, brighty, 3);

      /* to print file, we'll have to use the GLOBAL
       parameter u!   */

      u = knot[i] + t * diff;
      curvatures[k]=h; //fprintf(outfile, "%2.3f  %f\n", u, h);
      k++;
    }

    for (t = 0.5; t < 0.999; t = t + delt) {
      subdiv(3, coeffx, t, bleftx, brightx);
      subdiv(3, coeffy, t, blefty, brighty);

      h = curvature_0(bleftx, blefty, 3);
      h = 0.0 - h;
      /* minus sign since order of polygon
       traversal is reversed!
       */
      u = knot[i] + t * diff;
      curvatures[k]=h; //fprintf(outfile, "%2.3f  %f\n", u, h);
      k++;
    }
  }
  //
  //  /* print very last kappa: */
  //  t=1.0;
  //  h=curvature_1(coeffx,coeffy,weight,3);
  //  fprintf(outfile,"%2.3f    %f\n",knot[l],h);
}

void CurveSmoother::bezier_kappas(double* bez_x, double* bez_y, int degree, int l, int dense, double* curvatures) {
  double bleftx[50],blefty[50],brightx[50],brighty[50];
  double coeffx[50],coeffy[50];

  int ldeg = degree * l;
  int num_curvatures = dense + 1;
  int curvature_index=0;
  double* res_segment = curvatures;

  double delt = 1.0 / (double) dense;

  for (int i = 0; i < ldeg; i += degree) {

    for (int k = 0; k <= degree; k++) {
      coeffx[k] = bez_x[i + k];
      coeffy[k] = bez_y[i + k];
    }


    for (double t = 0.0; t < 0.5; t = t + delt) {
      subdiv(degree, coeffx, t, bleftx, brightx);
      subdiv(degree, coeffy, t, blefty, brighty);

      curvatures[curvature_index]=curvature_0(brightx, brighty, degree);
      curvature_index++;
    }

    for (double t = 0.5; t < 0.999; t = t + delt) {
      subdiv(degree, coeffx, t, bleftx, brightx);
      subdiv(degree, coeffy, t, blefty, brighty);

      /* minus sign since order of polygon
       traversal is reversed!
       */
      curvatures[curvature_index] = - curvature_0(bleftx, blefty, degree);
      curvature_index++;
    }
    res_segment += num_curvatures;
  }
  //
  //  /* print very last kappa: */
  //  t=1.0;
  //  h=curvature_1(coeffx,coeffy,weight,3);
  //  fprintf(outfile,"%2.3f    %f\n",knot[l],h);
}

/* computes curvature of Bezier curve at t=0 */

double CurveSmoother::curvatureAt0(const std::vector<CurvePoint>& p, size_t idx, int32_t degree) {
double b0[2], b1[2], b2[2];
double dist;

b0[0] = p[idx].x;
b0[1] = p[idx].y;
idx++;
b1[0] = p[idx].x;
b1[1] = p[idx].y;
idx++;
b2[0] = p[idx].x;
b2[1] = p[idx].y;
idx++;

dist = sqrt((b1[0] - b0[0]) * (b1[0] - b0[0]) + (b1[1] - b0[1]) * (b1[1] - b0[1]));

return (2.0 * (degree - 1) * area(b0, b1, b2) / (degree * dist * dist * dist));
}

double CurveSmoother::curvature_0(double* bez_x, double* bez_y, int degree) {
  double b0[2], b1[2], b2[2];
  double dist;

  b0[0] = bez_x[0];
  b1[0] = bez_x[1];
  b2[0] = bez_x[2];
  b0[1] = bez_y[0];
  b1[1] = bez_y[1];
  b2[1] = bez_y[2];

  dist = sqrt((b1[0] - b0[0]) * (b1[0] - b0[0]) + (b1[1] - b0[1]) * (b1[1] - b0[1]));

  return (2.0 * (degree - 1) * area(b0, b1, b2) / (degree * dist * dist * dist));
}

/*
  subdivides bezier curve at parameter value t.
  Output: left and right polygon with respective weights.
  Ordering of right polygon is reversed.
*/
void CurveSmoother::subdiv(int degree, double* coeff, double t, double* bleft, double* bright) {
  int r,i;
  double t1;

  t1 = 1.0 - t;

/*
  first, obtain right subpolygon from rat de Casteljau
*/

  for (i=0;i <= degree; i++) bright[i] = coeff[i];

  for (r=1; r<= degree; r++)
  for (i=0; i<= degree - r; i++) {
    bright[i]= ( t1 * bright[i] +  t * bright[i+1] );
  }

/*
  use same as above in order to get left half. Idea:
  reverse ordering; then the above yields left half.
 */

  t = 1.0 - t; t1 = 1.0 - t;
  for (i=0;i <= degree; i++) bleft[degree-i] = coeff[i];

  for (r=1; r<= degree; r++)
  for (i=0; i<= degree - r; i++) {
    bleft[i]= ( t1 * bleft[i] +  t * bleft[i+1] );
  }

}

/* converts  cubic B-spline polygon into piecewise Bezier polygon
Input:  bspl: B-spline control polygon
  knot: knot sequence
  l:    no. of intervals

Output: bez:  piecewise Bezier polygon

*/

void CurveSmoother::bspline_to_bezier(double* bspl, double* knot, int l, double* bez) {
  int i, i3, l3;
/* The first points need special attention:  */

  bez[0]=bspl[0];
  bez[1]=bspl[1];
  if(l>1)
  {
    bez[2]=((knot[2]-knot[1])*bspl[1]
    +(knot[1]-knot[0])*bspl[2]    )/(knot[2]-knot[0]);
  }
  if(l>2)
  {
    bez[4]=( (knot[3]-knot[1])*bspl[2]
          +(  knot[1]-knot[0])*bspl[3]  )/(knot[3]-knot[0]);
    bez[3]=( (knot[2]-knot[1])*bez[2]
          + (knot[1]-knot[0])*bez[4]    )/(knot[2]-knot[0]);
  }
/* Now the main part:   */

  for(i=2; i<l-1; i++)
  {
  i3=3*i;
  bez[i3-1]=( (knot[i+1]-knot[i])*bspl[i]
           +  (knot[i]-knot[i-2])*bspl[i+1]    )/(knot[i+1]-knot[i-2]);

  bez[i3+1]=( (knot[i+2]-knot[i])*bspl[i+1]
           +(  knot[i]-knot[i-1])*bspl[i+2]  )/(knot[i+2]-knot[i-1]);

  bez[i3]  =( (knot[i+1]-knot[i])*bez[i3-1]
      + (knot[i]-knot[i-1])*bez[i3+1]  )/(knot[i+1]-knot[i-1]);
  }

/* The last points need special attention:  */

  l3=l*3;
  if(l>2)
  {
  bez[l3-4]=( (knot[l]-knot[l-1])*bspl[l-1]
           +  (knot[l-1]-knot[l-3])*bspl[l]  )/(knot[l]-knot[l-3]);
  }
  if(l>1)
  {
  bez[l3-2]=( (knot[l]-knot[l-1])*bspl[l]
     +  (knot[l-1]-knot[l-2])*bspl[l+1]    )/(knot[l]-knot[l-2]);

  bez[l3-3]=( (knot[l]-knot[l-1])*bez[l3-4]
     + (knot[l-1]-knot[l-2])*bez[l3-2]   )/(knot[l]-knot[l-2]);
  }
  bez[l3-1]= bspl[l+1];
  bez[l3]  = bspl[l+2];
}

//void CurveSmoother::bezier2points(int degree, int l, const double* bez, int dense, double* points, int* point_num) {
//  int i;
//  double* ergp = NULL;
//
//  // for each segment, convert to points  and plot:
//  ergp = points;
//  *point_num = 0;
//  for (i = 0; i < degree * l; i += degree) {
//    bez_to_points(degree, dense, &bez[i], ergp);
//
////    ergp += (dense);
////    *point_num += (dense);
//    ergp+=(dense+1);
//    *point_num+=(dense+1);
//  }
//}

void CurveSmoother::bezier2points(int degree, int l, const double* bez, int dense, double* points, int* point_num) {

  double aux[50];
  double res_tmp[1000];

  int ldeg = degree * l;
  int num_bez_points = dense + 1;
  double* res_segment = points;

  *point_num = 0;
  for (int i = 0; i < ldeg; i += degree) {

    memset(aux, 255, 50*sizeof(double));

    for (int k = 0; k <= degree; k++) {
      aux[k] = bez[i + k];
    }

    memset(res_tmp, 255, 1000*sizeof(double));

    bez_to_points(degree, dense, aux, res_tmp);
    memcpy(res_segment, res_tmp, num_bez_points*sizeof(double));

    *point_num += num_bez_points;
    res_segment += num_bez_points;
//    bez_to_points(degree, dense, aux, res_segment);
//    *point_num += num_bez_points;
//    res_segment += num_bez_points;

//    fprintf(psfile, "%f %f moveto\n", scale_x * (points_x[0] - value[0]), scale_y * (points_y[0] - value[2]));
//
//    for (j = 1; j <= dense; j++) {
//      fprintf(psfile, "%f %f lineto\n", scale_x * (points_x[j] - value[0]), scale_y * (points_y[j] - value[2]));
//    }
  }
}

// Converts Bezier curve into point sequence
//
// Input:   degree:  degree of curve.
// npoints: # of coordinates to be generated. (counting from 0!)
// coeff:   coordinates of control polygon.
// Output:  points:  coordinates of points on curve.

void CurveSmoother::bez_to_points(int degree, int npoints, const double* coeff, double* points) {
  int i;

  if (npoints < 2) return;
  npoints--;

  double delt = 1.0 / (double) npoints;
  double t = 0.0;

  for (i = 0; i <= npoints; i++) {
    points[i] = hornbez(degree, coeff, t);
    t = t + delt;
  }
}

void CurveSmoother::bez_to_points2(const double* coeff, double delta_s0, int degree, uint32_t& npoints, double* points) {
  double const eps_s = 500;//0.005;

  if (npoints < 2) return;
  npoints--;

  double delt = 1.0 / (double) npoints;
  double last_t = 0;

  points[0] = coeff[0];
  double last_val = hornbez(degree, coeff, 1);
//  double len = last_val - points[0];
//  double delta_s0 = len/npoints;

  double t=delt;
  for (uint32_t i = 1; i < npoints; i++) {
    bool close_enough = false;
    double cdelt=delt;
    printf("point %i/%u\n",i, npoints-1);
    while(!close_enough) {
      if(t>=1.0) {break;}
      points[i] = hornbez(degree, coeff, t);
      double delta_s = std::abs(points[i]-points[i-1]);
      if(delta_s - delta_s0 > eps_s) {cdelt /= 2; t=last_t;}
      else if(delta_s - delta_s0 < -eps_s) {}
      else {close_enough=true;}
      last_t = t;
      t = t + cdelt;
    }
    printf("%u. t=%f, val@t: %f, delta_s: %f, delta0: %f\n",i, t, points[i], std::abs(points[i]-points[i-1]), delta_s0);
    if(t>=1) {
      points[i]=last_val;
      npoints = i+1;
      return;
    }
  }
}

// uses  Horner's scheme to compute one coordinate
// value of a  Bezier curve. Has to be called
// for each coordinate  (x,y, and/or z) of a control polygon.
// Input:   degree: degree of curve.
// coeff:  array with coefficients of curve.
// t:      parameter value.
// Output: coordinate value.

double CurveSmoother::hornbez(int degree, const double* coeff, double t) {
  int i;
  int n_choose_i;
  double fact, t1, aux;

  t1 = 1.0 - t;
  fact = 1.0;
  n_choose_i = 1;

  aux = coeff[0] * t1;
  for (i = 1; i < degree; i++) {
    fact = fact * t;
    n_choose_i = n_choose_i * (degree - i + 1) / i;
    aux = (aux + fact * n_choose_i * coeff[i]) * t1;
  }

  aux = aux + fact * t * coeff[degree];

  return aux;
}

CurveSmoother::point2d CurveSmoother::hornbez(int degree, const CurveSmoother::point2d* p, double t) {
  int i;
  int n_choose_i;
  double fact, t1;
  point2d aux;

  t1 = 1.0 - t;
  fact = 1.0;
  n_choose_i = 1;

  aux.x = p[0].x * t1;
  aux.y = p[0].y * t1;
  for (i = 1; i < degree; i++) {
    fact = fact * t;
    n_choose_i = n_choose_i * (degree - i + 1) / i;
    aux.x = (aux.x + fact * n_choose_i * p[i].x) * t1;
    aux.y = (aux.y + fact * n_choose_i * p[i].y) * t1;
  }

  aux.x = aux.x + fact * t * p[degree].x;
  aux.y = aux.y + fact * t * p[degree].y;

  return aux;
}

// find area of 2D triangle p1,p2,p3
double CurveSmoother::area(double* p1, double* p2,double* p3) {
  return((p2[0]-p1[0])*(p3[1]-p1[1])-(p2[1]-p1[1])*(p3[0]-p1[0]))/2.0;
}

void CurveSmoother::derivative(double t, double* px, double* py, double& x, double& y) const
{
  x = 3*(px[3]-3*px[2]+3*px[1]-px[0])*t*t + 2*(3*px[2]-6*px[1]+3*px[0])*t + 3*(px[1]-px[0]);
  y = 3*(py[3]-3*py[2]+3*py[1]-py[0])*t*t + 2*(3*py[2]-6*py[1]+3*py[0])*t + 3*(py[1]-py[0]);
}

void CurveSmoother::derivative2(double t, double* px, double* py, double& x, double& y) const
{
  x = 6*(px[3]-3*px[2]+3*px[1]-px[0])*t + 2*(3*px[2]-6*px[1]+3*px[0]);
  y = 6*(py[3]-3*py[2]+3*py[1]-py[0])*t + 2*(3*py[2]-6*py[1]+3*py[0]);
}

inline void CurveSmoother::sampleLineLinearEquidist(std::vector<CurvePoint>::const_iterator pit0, std::vector<
    CurvePoint>::const_iterator pit1, double point_dist, std::vector<CurvePoint>& linepoints) {

  const double& x0 = (*pit0).x;
  const double& y0 = (*pit0).y;
  const double& x1 = (*pit1).x;
  const double& y1 = (*pit1).y;

  double len = hypot(x1 - x0, y1 - y0);

  // minimum is to return the input points
  uint32_t num_points = std::max(2u, uint32_t(len / point_dist + 1.5));

  double delta_x = (x1 - x0) / (num_points-1);
  double delta_y = (y1 - y0) / (num_points-1);

//  double num_points_f = std::max(2., len / point_dist+1);
//  double last_dist = hypot(x1 - (x0 + (num_points - 1) * delta_x), y1 - (y0 + (num_points - 1) * delta_y));
//  double delta_s = hypot(delta_x, delta_y);
//  printf("delta_s: %f, point_dist: %f, num_points: %u, num_points_f: %f, last_dist: %f\n", delta_s, point_dist, num_points, num_points_f, last_dist);

  for (uint32_t j = 0; j < num_points; j++) {
    cp_.x = x0 + j * delta_x;
    cp_.y = y0 + j * delta_y;
    linepoints.push_back(cp_);
  }
}

void CurveSmoother::sampleLinearEquidist(std::vector<CurvePoint>& points, std::vector<bool>& ignore, double point_dist,
                                         std::vector<CurvePoint>* linepoints) {
  linepoints->clear();
  if (points.size() < 2) {
    return;
  }

  std::vector<CurvePoint>::const_iterator pit0 = points.begin();
  std::vector<CurvePoint>::const_iterator pit1 = pit0+1;
  std::vector<bool>::const_iterator iit = ignore.begin();

  for (; pit1 != points.end(); pit0++, pit1++, iit++) {
    if(*iit) {
      linepoints->push_back((*pit0));
      continue;
    }
    sampleLineLinearEquidist(pit0, pit1, point_dist, *linepoints);
    if((pit1+1) != points.end()) {linepoints->pop_back();}
  }
}

void CurveSmoother::sampleCubicBezierPoints(int npoints, const point2d* control_polygon, double& s, std::vector<CurvePoint>& sampled_points) {
  int i;

  if (npoints < 2) {return;}
  npoints--;

  double delt = 1.0 / (double) npoints;
  double t = 0.0;

  double coeff_x[4], bleft_x[4], bright_x[4];
  double coeff_y[4], bleft_y[4], bright_y[4];

  coeff_x[0] = control_polygon[0].x; coeff_y[0] = control_polygon[0].y;
  coeff_x[1] = control_polygon[1].x; coeff_y[1] = control_polygon[1].y;
  coeff_x[2] = control_polygon[2].x; coeff_y[2] = control_polygon[2].y;
  coeff_x[3] = control_polygon[3].x; coeff_y[3] = control_polygon[3].y;

  CurvePoint cp;
  cp.kappa_prime = 0;

  double last_ds=0;
  for (i = 0; i <= npoints; i++) {
    point2d point_sample = hornbez(3, control_polygon, t);

    cp.x = point_sample.x;
    cp.y = point_sample.y;

    // subdivide at t to calculate curvature
    subdiv(3, coeff_x, t, bleft_x, bright_x);
    subdiv(3, coeff_y, t, bleft_y, bright_y);


    if(t < 0.5)  {
      cp.theta = atan2(bright_y[1]-bright_y[0], bright_x[1]-bright_x[0]);
      cp.kappa = curvature_0(bright_x, bright_y, 3);
    }
    else {
      // minus sign since order of polygon traversal is reversed!
      cp.theta = atan2(bleft_y[0]-bleft_y[1], bleft_x[0]-bleft_x[1]);
      cp.kappa = curvature_0(bleft_x, bleft_y, 3);
      cp.kappa = 0.0 - cp.kappa;
    }

   point2d p1, p2, p3, p4;
   p4.x = bleft_x[0];    p4.y = bleft_y[0];
   p3.x = bleft_x[1];    p3.y = bleft_y[1];
   p2.x = bleft_x[2];    p2.y = bleft_y[2];
   p1.x = bleft_x[3];    p1.y = bleft_y[3];

//   printf("p4: (%f, %f) <-> cp: (%f, %f)\n", p4.x, p4.y, cp.x, cp.y);
   double ds = BezierArcLength(p1, p2, p3, p4);
   double dds=ds-last_ds;
   last_ds=ds;
   s+=dds;
   cp.s = s;

    sampled_points.push_back(cp);
    t = t + delt;
  }
}


void CurveSmoother::sampleCubicBezierEquidist(const std::vector<CurvePoint>& control_polygon, double point_dist, std::vector<CurvePoint>& sampled_points) {

  double s=0;
  point2d p[4];
  sampled_points.clear();

  for(uint32_t i3 = 0; i3 < (uint32_t)control_polygon.size()-1; i3+=3) {

      // get arc length of original Bezier interval to see how many points
      // we have to sample
    p[0].x = control_polygon[i3].x;    p[0].y = control_polygon[i3].y;
    p[1].x = control_polygon[i3+1].x;    p[1].y = control_polygon[i3+1].y;
    p[2].x = control_polygon[i3+2].x;    p[2].y = control_polygon[i3+2].y;
    p[3].x = control_polygon[i3+3].x;    p[3].y = control_polygon[i3+3].y;
    double ds = BezierArcLength(p[0], p[1], p[2], p[3]);

        // minimum is to return the input points
    uint32_t num_points = std::max(2u, uint32_t(ds / point_dist + 1.5));

    sampleCubicBezierPoints(num_points - 1, p, s, sampled_points);

      // remove last point since it's too close to next interval's start point
    if(i3 != control_polygon.size()-4) {
      sampled_points.pop_back();
 //     s = (*(--sampled_points.end())).s;
    }
  }
}

bool CurveSmoother::clothoideSpline(std::vector<CurvePoint>& points, double theta0, double kappa0,
                                    double s, int n_lookahead, std::vector<CurvePoint>* splinepoints) {
  int i, j, k;

  double dkappa=0;
  double errorFactor;
  double term1, term2;
  double x0_next_ccs, y0_next_ccs, theta0_next_ccs;
  double x0_next, y0_next, theta0_next, kappa0_next;
  double x_ccs, y_ccs, theta_ccs, kappa_ccs; // for points at the end

  double x0 = points[0].x; // Take first point for initialization
  double y0 = points[0].y;

  int num_points = points.size();

  if ((num_points <= 1) || (n_lookahead > num_points) || n_lookahead > (int)cs_temp_buf_size_) {
    return false;
  }

  splinepoints->clear();
  cp_.s = s;
  cp_.x = points[0].x;
  cp_.y = points[0].y;
  cp_.theta = normalizeAngle(theta0);
  cp_.kappa = kappa0;
  splinepoints->push_back(cp_);

  bool curve_end=false;
  for (i = 0; i <= num_points - n_lookahead - 1; i++) {
  if(i>=num_points-15 && !curve_end) {
    curve_end=true;
    n_lookahead=4;
  }
    //    for(j=0; j<8; j++) {
//      if(points[i + j].kappa> .005) {n_lookahead=4;} else {n_lookahead=8;}
//    }
    // Transform to CS of next point
    for (j = 0; j < n_lookahead; j++) {
      lookahead_x_[j] = cos(-theta0) * (points[i + 1 + j].x - x0) - sin(-theta0) * (points[i + 1 + j].y - y0);
      lookahead_y_[j] = sin(-theta0) * (points[i + 1 + j].x - x0) + cos(-theta0) * (points[i + 1 + j].y - y0);
    }

    // Least Squares

    // dkappa/ds = 6/sum(xi^6)*sum(xi^3*(yi-kappa0*xi^2/2))
    // sum xi^6
    double sum1 = 0, sum2 = 0;
    for (k = 0; k < n_lookahead; k++) {
      sum1 += lookahead_x_[k] * lookahead_x_[k] * lookahead_x_[k] * lookahead_x_[k] * lookahead_x_[k] * lookahead_x_[k];
      sum2 += (lookahead_x_[k] * lookahead_x_[k] * lookahead_x_[k]) * (lookahead_y_[k] - (kappa0 * (lookahead_x_[k] * lookahead_x_[k]) / 2.));
    }

    dkappa = 6 / sum1 * sum2;

    // Calculate smoothed values for next point

    // Calculate coordinates of next point in current CS
    x0_next_ccs = lookahead_x_[0];
    y0_next_ccs = 0.5 * kappa0 * (x0_next_ccs * x0_next_ccs) + 1.0 / 6 * dkappa * (x0_next_ccs * x0_next_ccs * x0_next_ccs); // f(x0_next_ccs)
    theta0_next_ccs = atan(kappa0 * x0_next_ccs + 0.5 * dkappa * (x0_next_ccs * x0_next_ccs)); // atan(dy/dx(x0_next))

    term1 = kappa0 * x0_next_ccs + .5 * dkappa * (x0_next_ccs * x0_next_ccs);
    term2 = 1.0 / (1 + term1 * term1);
    errorFactor = sqrt(term2 * term2 * term2); // 1/(1 + (kappa0*xNext) + .5*dkappa*xNext^2)^2)^(3/2)
    kappa0_next = errorFactor * (dkappa * x0_next_ccs + kappa0);

    // Calculate coordinates of next point in original CS
    x0_next = (cos(theta0) * x0_next_ccs - sin(theta0) * y0_next_ccs) + x0; // Rotate then translate back
    y0_next = (sin(theta0) * x0_next_ccs + cos(theta0) * y0_next_ccs) + y0;
    theta0_next = theta0 + theta0_next_ccs;

    // Store values for next loop
    x0 = x0_next;
    y0 = y0_next;
    theta0 = theta0_next;
    kappa0 = kappa0_next;

    // Store values for return
    double dx = x0_next - cp_.x;
    double dy = y0_next - cp_.y;
    cp_.s += sqrt(dx*dx + dy*dy);
    cp_.x = x0_next;
    cp_.y = y0_next;
    cp_.theta = normalizeAngle(theta0_next);
    cp_.kappa = kappa0_next;
    splinepoints->push_back(cp_);
  }

  // Change values back for calculation of the remaining points
  x0 = (*splinepoints)[num_points - n_lookahead - 1].x;
  y0 = (*splinepoints)[num_points - n_lookahead - 1].y;
  theta0 = (*splinepoints)[num_points - n_lookahead - 1].theta;
  kappa0 = (*splinepoints)[num_points - n_lookahead - 1].kappa;

  // Calculate values for last (n_lookahead-2) points (values in the last clothoid)
  for (i = num_points - n_lookahead + 1; i < num_points; i++) {
    // Calculate coordinates of next point in current CS
    x_ccs = lookahead_x_[i - (num_points - n_lookahead + 1) + 1]; // From main for
    y_ccs = 0.5 * kappa0 * (x_ccs * x_ccs) + 1 / 6 * dkappa * (x_ccs * x_ccs * x_ccs); // f(x0_next)
    theta_ccs = atan(kappa0 * x_ccs + 0.5 * dkappa * (x_ccs * x_ccs)); // atan(dy/dx(x0_next))

    term1 = kappa0 * x_ccs + .5 * dkappa * (x_ccs * x_ccs);
    term2 = 1.0 / (1 + term1 * term1);
    errorFactor = sqrt(term2 * term2 * term2); // 1/(1 + (kappa0*xNext) + .5*dkappa*xNext^2)^2)^(3/2)
    kappa_ccs = errorFactor * (dkappa * x_ccs + kappa0);

    // Calculate coordinates in original CS
    double x_new = (cos(theta0) * x_ccs - sin(theta0) * y_ccs) + x0; // Rotate then translate back
    double y_new = (sin(theta0) * x_ccs + cos(theta0) * y_ccs) + y0;
    double dx = x_new - cp_.x;
    double dy = y_new - cp_.y;
    cp_.s += sqrt(dx*dx + dy*dy);
    cp_.x = x_new;
    cp_.y = y_new;
    cp_.theta = normalizeAngle(theta0 + theta_ccs);
    cp_.kappa = kappa_ccs;
    splinepoints->push_back(cp_);
  }

  return true;
}

//---------------------------------------------------------------------------
// NOTES:       TOLERANCE is a maximum error ratio
//                      if n_limit isn't a power of 2 it will be act like the next higher
//                      power of two.
//double CurveSmoother::Simpson(double(*f)(double), double a, double b, int n_limit, double tolerance) {
double CurveSmoother::Simpson(double a, double b, int n_limit, double tolerance) {
  int n = 1;
  double multiplier = (b - a) / 6.0;
  double endsum = BezierArcLengthFunction(a) + BezierArcLengthFunction(b);
  double interval = (b - a) / 2.0;
  double asum = 0;
  double bsum = BezierArcLengthFunction(a + interval);
  double est1 = multiplier * (endsum + 2 * asum + 4 * bsum);
  double est0 = 2 * est1;

  while (n < n_limit && (std::abs(est1) > 0 && std::abs((est1 - est0) / est1) > tolerance)) {
    n *= 2;
    multiplier /= 2;
    interval /= 2;
    asum += bsum;
    bsum = 0;
    est0 = est1;
    double interval_div_2n = interval / (2.0 * n);

    for (int i = 1; i < 2 * n; i += 2) {
      double t = a + i * interval_div_2n;
      bsum += BezierArcLengthFunction(t); //f(t);
    }

    est1 = multiplier * (endsum + 2 * asum + 4 * bsum);
  }

  return est1;
}

//
//---------------------------------------------------------------------------
//
double CurveSmoother::BezierArcLength(point2d p1, point2d p2, point2d p3, point2d p4) {
  point2d k1, k2, k3, k4;

  k1 = p1 * (-1) + (p2 - p3) * 3 + p4;
  k2 = (p1 + p3) * 3 - p2 * 6;
  k3 = (p2 - p1) * 3;
  k4 = p1;

  q1_ = 9.0 * (k1.x*k1.x + k1.y*k1.y);
  q2_ = 12.0 * (k1.x * k2.x + k1.y * k2.y);
  q3_ = 3.0 * (k1.x * k3.x + k1.y * k3.y) + 4.0 * (k2.x*k2.x + k2.y*k2.y);
  q4_ = 4.0 * (k2.x * k3.x + k2.y * k3.y);
  q5_ = k3.x*k3.x + k3.y*k3.y;

  return Simpson(0, 1, 1024, 0.001);
}

} // namespace vlr

