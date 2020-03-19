#include "ct-view.h"
#include "ct-widget.h"

QControllerView::QControllerView( QWidget *parent, const char *name )
  : QWidget( parent, name )
{ 
  QGridLayout     * box;
  QPen              pen;

  box = new QGridLayout( this, 2, 2);

  ctview = new QCtWidget(this);
  box->addWidget( ctview, 1, 1 );

  for (int i=0; i<300; i++) {
    ctview->ct_steering_set_x[i]  = 0.0;
    ctview->ct_steering_set_y[i]  = 0.0;
    ctview->ct_steering_get_x[i]  = 0.0;
    ctview->ct_steering_get_y[i]  = 0.0;

    ctview->ct_velocity_set_x[i]  = 0.0;
    ctview->ct_velocity_set_y[i]  = 0.0;
    ctview->ct_velocity_get_x[i]  = 0.0;
    ctview->ct_velocity_get_y[i]  = 0.0;

    ctview->pa_steering_ct_x[i]  = 0.0;
    ctview->pa_steering_ct_y[i]  = 0.0;
    ctview->pa_steering_torque_x[i]  = 0.0;
    ctview->pa_steering_torque_y[i]  = 0.0;
    ctview->pa_steering_angle_x[i]  = 0.0;
    ctview->pa_steering_angle_y[i]  = 0.0;

    ctview->pa_throttle_set_x[i]  = 0.0;
    ctview->pa_throttle_set_y[i]  = 0.0;
    ctview->pa_throttle_get_x[i]  = 0.0;
    ctview->pa_throttle_get_y[i]  = 0.0;

    ctview->pa_brake_set_x[i]  = 0.0;
    ctview->pa_brake_set_y[i]  = 0.0;
    ctview->pa_brake_get_x[i]  = 0.0;
    ctview->pa_brake_get_y[i]  = 0.0;

  }

  /*************************** CONTROLLER ***********************************/
  ctview->ct_steering->setAutoLegend(TRUE);
  ctview->ct_steering->setLegendPosition(QwtPlot::Bottom);

  ctview->ct_steering_set_curve = 
    ctview->ct_steering->insertCurve("  CONTROLLER TARGET ANGLE", QwtPlot::xBottom, QwtPlot::yLeft);
  ctview->ct_steering_get_curve =
    ctview->ct_steering->insertCurve("  CAN LISTENER ANGLE", QwtPlot::xBottom, QwtPlot::yLeft);
  
  ctview->ct_steering->setAxisScale(QwtPlot::yLeft, -800.0, 800.0, 200.0);
  // ctview->ct_steering->setAxisTitle(QwtPlot::yLeft,"Degrees");
  ctview->ct_steering->setAxisScale(QwtPlot::xBottom, -6.0, 0.0, 1.0);
  ctview->ct_steering->enableAxis(QwtPlot::xBottom,10.0);
  
  pen.setColor(Qt::red);
  pen.setWidth(2);
  ctview->ct_steering->setCurvePen( ctview->ct_steering_set_curve, pen );
  ctview->ct_steering->setCurveData( ctview->ct_steering_set_curve, 
				     ctview->ct_steering_set_x, 
				     ctview->ct_steering_set_y, 300 );
  
  pen.setColor(Qt::blue);
  pen.setWidth(2);
  ctview->ct_steering->setCurvePen( ctview->ct_steering_get_curve, pen );
  ctview->ct_steering->setCurveData( ctview->ct_steering_get_curve, 
				     ctview->ct_steering_get_x,
				     ctview->ct_steering_get_y, 300 );

  ctview->ct_velocity->setAutoLegend(TRUE);
  ctview->ct_velocity->setLegendPosition(QwtPlot::Bottom);

  ctview->ct_velocity_set_curve = 
    ctview->ct_velocity->insertCurve("  CONTROLLER TARGET", QwtPlot::xBottom, QwtPlot::yLeft);
  ctview->ct_velocity_get_curve =
    ctview->ct_velocity->insertCurve("  CAN LISTENER", QwtPlot::xBottom, QwtPlot::yLeft);
  
  ctview->ct_velocity->setAxisScale(QwtPlot::yLeft, -3.0, 15.0, 2.0);
  //  ctview->ct_velocity->setAxisTitle(QwtPlot::yLeft,"m/s");
  ctview->ct_velocity->setAxisScale(QwtPlot::xBottom, -6.0, 0.0, 1.0);
  ctview->ct_velocity->enableAxis(QwtPlot::xBottom,1);

  pen.setColor(Qt::red);
  pen.setWidth(2);
  ctview->ct_velocity->setCurvePen( ctview->ct_velocity_set_curve, pen );
  ctview->ct_velocity->setCurveData( ctview->ct_velocity_set_curve, 
				     ctview->ct_velocity_set_x, 
				     ctview->ct_velocity_set_y, 300 );
  
  pen.setColor(Qt::blue);
  pen.setWidth(2);
  ctview->ct_velocity->setCurvePen( ctview->ct_velocity_get_curve, pen );
  ctview->ct_velocity->setCurveData( ctview->ct_velocity_get_curve, 
				     ctview->ct_velocity_get_x, 
				     ctview->ct_velocity_get_y, 300 );
  
  /*************************** PASSAT ***********************************/
  ctview->pa_steering->setAutoLegend(TRUE);
  ctview->pa_steering->setLegendPosition(QwtPlot::Bottom);

  ctview->pa_steering_ct_curve =
    ctview->pa_steering->insertCurve("  CONTROLLER TARGET ANGLE", QwtPlot::xBottom, QwtPlot::yLeft);
  ctview->pa_steering_angle_curve =
    ctview->pa_steering->insertCurve("  CAN LISTENER ANGLE", QwtPlot::xBottom, QwtPlot::yLeft);
  ctview->pa_steering_torque_curve = 
    ctview->pa_steering->insertCurve("  PASSAT TORQUE", QwtPlot::xBottom, QwtPlot::yLeft);

  ctview->pa_steering->setAxisScale(QwtPlot::yLeft, -800.0, 800.0, 200.0);
  // ctview->pa_steering->setAxisTitle(QwtPlot::yLeft,"Degrees");
  ctview->pa_steering->setAxisScale(QwtPlot::xBottom, -6.0, 0.0, 1.0);
  ctview->pa_steering->enableAxis(QwtPlot::xBottom,1);

  pen.setColor(Qt::red);
  pen.setWidth(2);
  ctview->pa_steering->setCurvePen( ctview->pa_steering_ct_curve, pen );
  ctview->pa_steering->setCurveData( ctview->pa_steering_ct_curve, 
				     ctview->pa_steering_ct_x,
				     ctview->pa_steering_ct_y, 300 );
  
  pen.setColor(Qt::blue);
  pen.setWidth(2);
  ctview->pa_steering->setCurvePen( ctview->pa_steering_angle_curve, pen );
  ctview->pa_steering->setCurveData( ctview->pa_steering_angle_curve, 
				     ctview->pa_steering_angle_x,
				     ctview->pa_steering_angle_y, 300 );

  pen.setColor(QColor(0,100,0));
  pen.setWidth(2);
  ctview->pa_steering->setCurvePen( ctview->pa_steering_torque_curve, pen );
  ctview->pa_steering->setCurveData( ctview->pa_steering_torque_curve, 
				     ctview->pa_steering_torque_x,
				     ctview->pa_steering_torque_y, 300 );
  

  ctview->pa_throttle->setAutoLegend(TRUE);
  ctview->pa_throttle->setLegendPosition(QwtPlot::Bottom);

  ctview->pa_throttle_set_curve = 
    ctview->pa_throttle->insertCurve("  PASSAT THROTTLE", QwtPlot::xBottom, QwtPlot::yLeft);
  ctview->pa_throttle_get_curve =
    ctview->pa_throttle->insertCurve("  CAN THROTTLE", QwtPlot::xBottom, QwtPlot::yLeft);

  ctview->pa_throttle->setAxisScale(QwtPlot::yLeft, 0.0, 100.0, 20.0);
  //  ctview->pa_throttle->setAxisTitle(QwtPlot::yLeft,"percent");
  ctview->pa_throttle->setAxisScale(QwtPlot::xBottom, -6.0, 0.0, 1.0);
  ctview->pa_throttle->enableAxis(QwtPlot::xBottom,1);

  pen.setColor(Qt::red);
  pen.setWidth(2);
  ctview->pa_throttle->setCurvePen( ctview->pa_throttle_set_curve, pen );
  ctview->pa_throttle->setCurveData( ctview->pa_throttle_set_curve, 
				     ctview->pa_throttle_set_x,
				     ctview->pa_throttle_set_y, 300 );
  
  pen.setColor(Qt::blue);
  pen.setWidth(2);
  ctview->pa_throttle->setCurvePen( ctview->pa_throttle_get_curve, pen );
  ctview->pa_throttle->setCurveData( ctview->pa_throttle_get_curve, 
				     ctview->pa_throttle_get_x,
				     ctview->pa_throttle_get_y, 300 );
  
  
  ctview->pa_brake->setAutoLegend(TRUE);
  ctview->pa_brake->setLegendPosition(QwtPlot::Bottom);

  ctview->pa_brake_set_curve = 
    ctview->pa_brake->insertCurve("  PASSAT BRAKE PRESSURE", QwtPlot::xBottom, QwtPlot::yLeft);
  ctview->pa_brake_get_curve =
    ctview->pa_brake->insertCurve("  CAN BRAKE PRESSURE", QwtPlot::xBottom, QwtPlot::yLeft);
  
  ctview->pa_brake->setAxisScale(QwtPlot::yLeft, 0.0, 40.0, 10.0);
  //  ctview->pa_brake->setAxisTitle(QwtPlot::yLeft,"units");
  ctview->pa_brake->setAxisScale(QwtPlot::xBottom, -6.0, 0.0, 1.0);
  ctview->pa_brake->enableAxis(QwtPlot::xBottom,1);

  pen.setColor(Qt::red);
  pen.setWidth(2);
  ctview->pa_brake->setCurvePen( ctview->pa_brake_set_curve, pen );
  ctview->pa_brake->setCurveData( ctview->pa_brake_set_curve, 
				  ctview->pa_brake_set_x,
				  ctview->pa_brake_set_y, 300 );
  
  pen.setColor(Qt::blue);
  pen.setWidth(2);
  ctview->pa_brake->setCurvePen( ctview->pa_brake_get_curve, pen );
  ctview->pa_brake->setCurveData( ctview->pa_brake_get_curve, 
				  ctview->pa_brake_get_x, 
				  ctview->pa_brake_get_y, 300 );
  
  ctview->ct_steering->replot();
  ctview->ct_velocity->replot();
  ctview->pa_steering->replot();
  ctview->pa_throttle->replot();
  ctview->pa_brake->replot();

  resize( 800, 800 );
  ctview->show();
}

void
QControllerView::addTargetMeasurement( dgc_controller_target_message *target,
				       dgc_can_status_message *can )
{
  static int first_time = 1;

  if (first_time) {
    for (int i=0; i<300; i++) {
      ctview->ct_steering_set_ts[i]  = target->timestamp;
      ctview->ct_steering_get_ts[i]  = can->timestamp;
      ctview->ct_steering_set_y[i]   = target->target_steering_angle;
      ctview->ct_steering_get_y[i]   = can->steering_angle;

      ctview->ct_velocity_set_ts[i]  = target->timestamp;
      ctview->ct_velocity_get_ts[i]  = can->timestamp;
      ctview->ct_velocity_set_y[i]   = target->target_velocity;
      ctview->ct_velocity_get_y[i]   =  (can->wheel_speed_fl + 
					 can->wheel_speed_fr  )/(3.6*2.0);
    }
    first_time = 0;
  }

  for (int i=0; i<300-1; i++) {
    ctview->ct_steering_set_ts[i]  = ctview->ct_steering_set_ts[i+1];
    ctview->ct_steering_set_y[i]   = ctview->ct_steering_set_y[i+1];
    ctview->ct_steering_get_ts[i]  = ctview->ct_steering_get_ts[i+1];
    ctview->ct_steering_get_y[i]   = ctview->ct_steering_get_y[i+1];

    ctview->ct_velocity_set_ts[i]  = ctview->ct_velocity_set_ts[i+1];
    ctview->ct_velocity_set_y[i]   = ctview->ct_velocity_set_y[i+1];
    ctview->ct_velocity_get_ts[i]  = ctview->ct_velocity_get_ts[i+1];
    ctview->ct_velocity_get_y[i]   = ctview->ct_velocity_get_y[i+1];
  }
  ctview->ct_steering_set_ts[299]  = target->timestamp;
  ctview->ct_steering_set_y[299]   = target->target_steering_angle;
  ctview->ct_steering_get_ts[299]  = can->timestamp;
  ctview->ct_steering_get_y[299]   = can->steering_angle;

  ctview->ct_velocity_set_ts[299]  = target->timestamp;
  ctview->ct_velocity_set_y[299]   = target->target_velocity;
  ctview->ct_velocity_get_ts[299]  = can->timestamp;
  ctview->ct_velocity_get_y[299]   = (can->wheel_speed_fl + 
				       can->wheel_speed_fr  )/(3.6*2.0);

  for (int i=0; i<300; i++) {
    ctview->ct_steering_get_x[i]  = 
      ctview->ct_steering_get_ts[i] - ctview->ct_steering_get_ts[299];
    ctview->ct_steering_set_x[i]  = 
      ctview->ct_steering_set_ts[i] - ctview->ct_steering_set_ts[299];

    ctview->ct_velocity_get_x[i]  = 
      ctview->ct_velocity_get_ts[i] - ctview->ct_velocity_get_ts[299];
    ctview->ct_velocity_set_x[i]  = 
      ctview->ct_velocity_set_ts[i] - ctview->ct_velocity_set_ts[299];
  }

  ctview->ct_steering->setCurveData( ctview->ct_steering_set_curve, 
				     ctview->ct_steering_set_x, 
				     ctview->ct_steering_set_y, 300 );
  ctview->ct_steering->setCurveData( ctview->ct_steering_get_curve, 
				     ctview->ct_steering_get_x, 
				     ctview->ct_steering_get_y, 300 );

  ctview->ct_velocity->setCurveData( ctview->ct_velocity_set_curve, 
				     ctview->ct_velocity_set_x, 
				     ctview->ct_velocity_set_y, 300 );
  ctview->ct_velocity->setCurveData( ctview->ct_velocity_get_curve, 
				     ctview->ct_velocity_get_x, 
				     ctview->ct_velocity_get_y, 300 );
  ctview->ct_steering->replot();
  ctview->ct_velocity->replot();
  ctview->update();
}

void
QControllerView::addActuatorMeasurement( dgc_passat_actuator_message *passat,
					 dgc_can_status_message *can,
					 dgc_controller_target_message *target )
{
  static int first_time = 1;

  if (first_time) {
    for (int i=0; i<300; i++) {
      ctview->pa_steering_torque_ts[i]  = passat->timestamp;
      ctview->pa_steering_angle_ts[i]   = can->timestamp;
      ctview->pa_steering_ct_ts[i]      = target->timestamp;
      ctview->pa_steering_torque_y[i]   = passat->steering_torque * 800;
      ctview->pa_steering_angle_y[i]    = can->steering_angle;
      ctview->pa_steering_ct_y[i]       = target->target_steering_angle;

      ctview->pa_throttle_set_ts[i]  = passat->timestamp;
      ctview->pa_throttle_get_ts[i]  = can->timestamp;
      ctview->pa_throttle_set_y[i]   = passat->throttle_fraction*100.0;
      ctview->pa_throttle_get_y[i]   = can->throttle_position;

      ctview->pa_brake_set_ts[i]     = passat->timestamp;
      ctview->pa_brake_get_ts[i]     = can->timestamp;
      ctview->pa_brake_set_y[i]      = passat->brake_pressure;
      ctview->pa_brake_get_y[i]      = can->brake_pressure;
    }
    first_time = 0;
  }

  for (int i=0; i<300-1; i++) {
    ctview->pa_steering_torque_ts[i] = ctview->pa_steering_torque_ts[i+1];
    ctview->pa_steering_torque_y[i]  = ctview->pa_steering_torque_y[i+1];
    ctview->pa_steering_angle_ts[i]  = ctview->pa_steering_angle_ts[i+1];
    ctview->pa_steering_angle_y[i]   = ctview->pa_steering_angle_y[i+1];
    ctview->pa_steering_ct_ts[i]     = ctview->pa_steering_ct_ts[i+1];
    ctview->pa_steering_ct_y[i]      = ctview->pa_steering_ct_y[i+1];

    ctview->pa_throttle_set_ts[i] = ctview->pa_throttle_set_ts[i+1];
    ctview->pa_throttle_set_y[i]  = ctview->pa_throttle_set_y[i+1];
    ctview->pa_throttle_get_ts[i] = ctview->pa_throttle_get_ts[i+1];
    ctview->pa_throttle_get_y[i]  = ctview->pa_throttle_get_y[i+1];

    ctview->pa_brake_set_ts[i]    = ctview->pa_brake_set_ts[i+1];
    ctview->pa_brake_set_y[i]     = ctview->pa_brake_set_y[i+1];
    ctview->pa_brake_get_ts[i]    = ctview->pa_brake_get_ts[i+1];
    ctview->pa_brake_get_y[i]     = ctview->pa_brake_get_y[i+1];
  }

  ctview->pa_steering_torque_ts[299] = passat->timestamp;
  ctview->pa_steering_torque_y[299]  = passat->steering_torque * 800;
  ctview->pa_steering_angle_ts[299]  = can->timestamp;
  ctview->pa_steering_angle_y[299]   = can->steering_angle;
  ctview->pa_steering_ct_ts[299]     = target->timestamp;
  ctview->pa_steering_ct_y[299]      = target->target_steering_angle;

  ctview->pa_throttle_set_ts[299] = passat->timestamp;
  if (passat->throttle_fraction>1)
    ctview->pa_throttle_set_y[299]  = 100.0;
  else
    ctview->pa_throttle_set_y[299]  = passat->throttle_fraction*100.0;
  ctview->pa_throttle_get_ts[299] = can->timestamp;
  ctview->pa_throttle_get_y[299]  = can->throttle_position;

  ctview->pa_brake_set_ts[299]    = passat->timestamp;
  ctview->pa_brake_set_y[299]     = passat->brake_pressure;
  ctview->pa_brake_get_ts[299]    = can->timestamp;
  ctview->pa_brake_get_y[299]     = can->brake_pressure;

  for (int i=0; i<300; i++) {
    ctview->pa_steering_torque_x[i]  = 
      ctview->pa_steering_torque_ts[i] - ctview->pa_steering_torque_ts[299];
    ctview->pa_steering_angle_x[i]  = 
      ctview->pa_steering_angle_ts[i] - ctview->pa_steering_angle_ts[299];
    ctview->pa_steering_ct_x[i]  = 
      ctview->pa_steering_ct_ts[i] - ctview->pa_steering_ct_ts[299];

    ctview->pa_throttle_set_x[i]  = 
      ctview->pa_throttle_set_ts[i] - ctview->pa_throttle_set_ts[299];
    ctview->pa_throttle_get_x[i]  = 
      ctview->pa_throttle_get_ts[i] - ctview->pa_throttle_get_ts[299];

    ctview->pa_brake_set_x[i]  = 
      ctview->pa_brake_set_ts[i] - ctview->pa_brake_set_ts[299];
    ctview->pa_brake_get_x[i]  = 
      ctview->pa_brake_get_ts[i] - ctview->pa_brake_get_ts[299];
  }

  ctview->pa_steering->setCurveData( ctview->pa_steering_ct_curve, 
				     ctview->pa_steering_ct_x, 
				     ctview->pa_steering_ct_y, 300 );
  ctview->pa_steering->setCurveData( ctview->pa_steering_torque_curve, 
				     ctview->pa_steering_torque_x, 
				     ctview->pa_steering_torque_y, 300 );
  ctview->pa_steering->setCurveData( ctview->pa_steering_angle_curve, 
				     ctview->pa_steering_angle_x, 
				     ctview->pa_steering_angle_y, 300 );

  ctview->pa_throttle->setCurveData( ctview->pa_throttle_set_curve, 
				     ctview->pa_throttle_set_x, 
				     ctview->pa_throttle_set_y, 300 );
  ctview->pa_throttle->setCurveData( ctview->pa_throttle_get_curve, 
				     ctview->pa_throttle_get_x, 
				     ctview->pa_throttle_get_y, 300 );

  ctview->pa_brake->setCurveData( ctview->pa_brake_set_curve, 
				  ctview->pa_brake_set_x, 
				  ctview->pa_brake_set_y, 300 );
  ctview->pa_brake->setCurveData( ctview->pa_brake_get_curve, 
				  ctview->pa_brake_get_x, 
				  ctview->pa_brake_get_y, 300 );

  ctview->pa_steering->replot();
  ctview->pa_throttle->replot();
  ctview->pa_brake->replot();
  ctview->update();
}

void
QControllerView::closeEvent( QCloseEvent *ev )
{
  ev = NULL;
  exit(0);
}

