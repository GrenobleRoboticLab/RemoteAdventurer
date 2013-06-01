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
    public class LinearBarIndicator:BarIndicator
    {
        public LinearBarIndicator()
        {
            DefaultStyleKey = typeof(LinearBarIndicator);
        }

        protected override void OnValueChanged(double newVal, double oldVal)
        {
            //every time the value changes set the width and the hight of
            //the indicator.
            //setting these properties will trigger the measure and arrange passes
            LinearScale scale = Owner as LinearScale;
            if (scale != null)
            {
                Size sz = GetIndicatorSize(scale.DesiredSize);
                Width = sz.Width;
                Height = sz.Height;
                scale.InvalidateMeasure();
            }
        }
        protected override Size MeasureOverride(Size availableSize)
        {
            //call the base version to set the owner
            base.MeasureOverride(availableSize);
            Size size = new Size();
            //get the desired size of the indicator based on the owner size
            LinearScale owner = Owner as LinearScale;
            if (owner != null)
            {
                size = GetIndicatorSize(availableSize);
            }
            return size;
        }
        public override void Arrange(Size finalSize)
        {
            LinearScale scale = Owner as LinearScale;
            if (scale != null)
            {
                Point pos = scale.GetIndicatorOffset(this);
                base.Arrange(new Rect(pos, DesiredSize));
            }
        }
        private Size GetIndicatorSize(Size availableSize)
        {//gets the size of the indicator based on the current value and the
            //owner dimensions
            LinearScale scale = Owner as LinearScale;
            double width = 0, height = 0;
            if (scale.Orientation == Orientation.Horizontal)
            {
                height = BarThickness;
                width = GetExtent(availableSize.Width, Value, scale.Maximum, scale.Minimum);
            }
            else
            {
                width = BarThickness;
                height = GetExtent(availableSize.Height, Value, scale.Maximum, scale.Minimum);
            }
            return new Size(width, height);
        }
        
        //gets the length the indicator should have
        private double GetExtent(double length, double value, double max, double min)
        {
            return length * (value - min) / (max - min);
        }

    }
}
