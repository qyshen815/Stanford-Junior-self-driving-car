#include "ht-view.h"
#include "ht-widget.h"

QHealthView::QHealthView( QWidget *parent, const char *name )
  : QWidget( parent, name )
{
  QGridLayout     * box;
  
  setCaption( "Healthmonitor View" );

  box = new QGridLayout( this, 2, 2);

  for (int i=0; i<NUM_MACHINES; i++) {
    htview[i] = new QHealthWidget( this );
    box->addWidget( htview[i], (i/2)+1, (i%2)+1 );
    hostname[i] = (char *) malloc( 10 * sizeof(char) );
    strncpy( hostname[i], "", 10 );
    htview[i]->hide();
  }

  inuse = 0;
  resize( 400, 200 );
}

void
QHealthView::initMachine( int num, char *name, int num_cpus, 
			  float min_temp, float max_temp, 
			  float min_volt, float max_volt,
			  float max_memory, float max_swap )
{
  QHealthWidget  * widget;
  QPen             pen;
  
  if (num<0 && num>3)
    return;

  widget = htview[num];

  widget->machine->setTitle(name); 
 
  switch(num_cpus) {
  case 1:
    widget->cpu2->hide();
    widget->cpu2Label->hide();
    widget->cpu3->hide();
    widget->cpu3Label->hide();
    widget->cpu4->hide();
    widget->cpu4Label->hide();
    break;
  case 2:
    widget->cpu3->hide();
    widget->cpu3Label->hide();
    widget->cpu4->hide();
    widget->cpu4Label->hide();
    break;
  case 3:
    widget->cpu4->hide();
    widget->cpu4Label->hide();
    break;
  default:
    break;
  }
  widget->cpu1->setMaxValue(100);
  widget->cpu2->setMaxValue(100);
  widget->cpu3->setMaxValue(100);
  widget->cpu4->setMaxValue(100);
  widget->memory->setMaxValue( max_memory );
  widget->swap->setMaxValue( max_swap );

  for (int i=0; i<50; i++) {
    widget->hdd_x[i]  = i; widget->hdd_y[i]  = 0.0;
    widget->core_x[i] = i; widget->core_y[i] = 0.0;
    widget->ps_x[i]   = i; widget->ps_y[i]   = 0.0;
    widget->volt_x[i] = i; widget->volt_y[i] = 0.0;
  }
  
  widget->temp->setAutoLegend(TRUE);
  widget->temp->setLegendPosition(QwtPlot::Bottom);
  
  widget->hdd_curve = 
    widget->temp->insertCurve("HDD", QwtPlot::xBottom, QwtPlot::yLeft);
  widget->core_curve =
    widget->temp->insertCurve("CPU Core", QwtPlot::xBottom, QwtPlot::yLeft);
  widget->ps_curve =
    widget->temp->insertCurve("DC/DC", QwtPlot::xBottom, QwtPlot::yLeft);
  widget->volt_curve = 
    widget->temp->insertCurve("Power", QwtPlot::xBottom, QwtPlot::yRight);

  pen.setColor(Qt::blue);
  pen.setWidth(5);
  widget->temp->setCurvePen( widget->hdd_curve, pen );
  widget->temp->setCurveData( widget->hdd_curve, widget->hdd_x, widget->hdd_y, 50 );

  pen.setColor(Qt::green);
  pen.setWidth(5);
  widget->temp->setCurvePen( widget->core_curve, pen );
  widget->temp->setCurveData( widget->core_curve, widget->core_x, widget->core_y, 50 );

  pen.setColor(Qt::red);
  pen.setWidth(5);
  widget->temp->setCurvePen( widget->ps_curve, pen );
  widget->temp->setCurveData( widget->ps_curve, widget->ps_x, widget->ps_y, 50 );
  
  pen.setColor(Qt::darkMagenta);
  pen.setStyle(Qt::DotLine);
  pen.setWidth(4);
  widget->temp->setCurvePen( widget->volt_curve, pen );
  widget->temp->setCurveData( widget->volt_curve, widget->volt_x, widget->volt_y, 50 );

  widget->temp->setAxisScale(QwtPlot::yLeft, min_temp, max_temp, 10.0);
  widget->temp->setAxisTitle(QwtPlot::yLeft,"Deg. C");

  widget->temp->setAxisScale(QwtPlot::yRight, min_volt, max_volt, 1.0);
  widget->temp->setAxisTitle(QwtPlot::yRight,"Volt");
  widget->temp->enableAxis(QwtPlot::yRight,1);

  widget->temp->replot();

  widget->process->setColumnWidthMode( 4, QListView::Maximum);
  widget->process->clear();

  widget->proc = (QListViewItem **) malloc( NUM_PROCESSES * 
					    sizeof(QListViewItem *) );
  for (int i=0; i<NUM_PROCESSES; i++) {
    widget->proc[i] = new QListViewItem( widget->process);
  }
   
  widget->temp->drawText( 100, 100, "Style: Sticks, Symbol: Ellipse");

  widget->show();
  widget->update();

  inuse++;
}

void
QHealthView::setTitle( int num, char *name )
{
  QHealthWidget  * widget;

  if (num<0 && num>3)
    return;
  widget = htview[num];

  widget->machine->setTitle(name); 
}

void
QHealthView::addMeasurement( int num, dgc_healthmon_status_message *status )
{
  QHealthWidget  * widget;
  int              i,j;

  if (num<0 && num>3)
    return;

  widget = htview[num];
  widget->load1->setText( QString().setNum( status->loadavg[0] ));
  widget->load2->setText( QString().setNum( status->loadavg[1] ));
  widget->load3->setText( QString().setNum( status->loadavg[2] ));
  for (i=0; i<status->num_cpus; i++) {
    switch(i) {
    case 0:
      widget->cpu1->setValue(status->cpu_usage[i]);
      break;
    case 1:
      widget->cpu2->setValue(status->cpu_usage[i]);
      break;
    case 2:
      widget->cpu3->setValue(status->cpu_usage[i]);
      break;
    case 3:
      widget->cpu4->setValue(status->cpu_usage[i]);
      break;
    default:
      break;
    }
  }
  widget->memory->setValue(status->memused);
  widget->swap->setValue(status->swapused);

  for(int i = 0; i < status->num_processes; i++) {
    if (i<NUM_PROCESSES) {
      j=(i+NUM_PROCESSES-1)%NUM_PROCESSES;
      widget->proc[j]->setText( 0, QString().setNum(status->process[i].cpu_usage) );
      widget->proc[j]->setText( 1, QString().setNum(status->process[i].mem_usage) );
      widget->proc[j]->setText( 2, QString().setNum(status->process[i].pid) );
      widget->proc[j]->setText( 3, QString(status->process[i].cmdline) );
    }
  }

  for (int i=50-1; i>0; i--) {
    widget->hdd_y[i]  = widget->hdd_y[i-1];
    widget->core_y[i] = widget->core_y[i-1];
    widget->ps_y[i]   = widget->ps_y[i-1];
    widget->volt_y[i]   = widget->volt_y[i-1];
  }
  widget->ps_y[0]    = status->ps_temp;
  widget->core_y[0]  = status->cpucoretemp;
  widget->hdd_y[0]   = status->max_hddtemp;
  widget->volt_y[0]   = status->vin;

  widget->temp->setCurveData( widget->hdd_curve, 
			      widget->hdd_x, widget->hdd_y, 50 );
  widget->temp->setCurveData( widget->core_curve, 
			      widget->core_x, widget->core_y, 50 );
  widget->temp->setCurveData( widget->ps_curve, 
			      widget->ps_x, widget->ps_y, 50 );
  widget->temp->setCurveData( widget->volt_curve, 
			      widget->volt_x, widget->volt_y, 50 );

  widget->temp->replot();
  widget->update();
}

void
QHealthView::closeEvent( QCloseEvent *ev )
{
  ev = NULL;
  exit(0);
}

