using System;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;
using System.Diagnostics;

namespace GaugeLib
{
    [TemplatePart(Name="PART_BAR",Type=typeof(Path))]
    public class RadialBarIndicator:BarIndicator
    {
        Path thePath;
        public RadialBarIndicator()
        {
            DefaultStyleKey = typeof(RadialBarIndicator);
        }

        public override void OnApplyTemplate()
        {
            base.OnApplyTemplate();
            thePath = GetTemplateChild("PART_BAR") as Path;
        }
        protected override void UpdateIndicatorOverride(Scale owner)
        {
            base.UpdateIndicatorOverride(owner);
            RadialScale scale = Owner as RadialScale;
            if (scale != null)
            {
                SetIndicatorGeometry(scale, Value);
            }
        }
        protected override Size MeasureOverride(Size availableSize)
        {
            //call the base version to set the parent
            base.MeasureOverride(availableSize);
            //return all the available size
            double width = 0, height = 0;
            if (!double.IsInfinity(availableSize.Width))
                width = availableSize.Width;
            if (!double.IsInfinity(availableSize.Height))
                height = availableSize.Height;
            RadialScale scale = Owner as RadialScale;
            if (scale != null)
            {
                //every time a resize happens the indicator needs to be redrawn
                SetIndicatorGeometry(scale, Value);
            }
            return new Size(width, height);
        }

        protected override Size ArrangeOverride(Size arrangeBounds)
        {
            TranslateTransform tt = new TranslateTransform();
            RadialScale scale = Owner as RadialScale;
            if (scale != null)
            {//calculate the geometry again. the first time this was done the owner had a size of (0,0)
                //and so did the indicator. once the owner has the correct size (measureOveride has been called)
                //i should re-calculate the shape of the indicator
                SetIndicatorGeometry(scale, Value);
                Point center = scale.GetIndicatorOffset();
                tt.X = center.X;
                tt.Y = center.Y;
                RenderTransform = tt;
            }
            return base.ArrangeOverride(arrangeBounds);
        }

        protected override void OnValueChanged(double newVal, double oldVal)
        {
            RadialScale scale = Owner as RadialScale;
            if (scale != null)
            {
                SetIndicatorGeometry(scale, Value);
            }
        }
        protected override void OnBarThicknesChanged(int newVal, int oldVal)
        {
            base.OnBarThicknesChanged(newVal, oldVal);
            RadialScale scale = Owner as RadialScale;
            if (scale != null)
            {
                SetIndicatorGeometry(scale, Value);
            }
        }

        //sets the indicator geometry based on the scale and the current value
        private void SetIndicatorGeometry(RadialScale scale, double value)
        {
            if (thePath != null)
            {
                double min = scale.MinAngle;
                double max = scale.GetAngleFromValue(Value);
                if (scale.SweepDirection == SweepDirection.Counterclockwise)
                {
                    min = -min;
                    max = -max;
                }
                double rad = scale.GetIndicatorRadius();
                if (rad > BarThickness)
                {
                    Geometry geom = RadialScaleHelper.CreateArcGeometry(min, max, rad, BarThickness, scale.SweepDirection);
                    //stop the recursive loop. only set a new geometry if it is different from the current one
                    if (thePath.Data == null || thePath.Data.Bounds != geom.Bounds)
                        thePath.Data = geom;
                }
            }
        }
    }
}
