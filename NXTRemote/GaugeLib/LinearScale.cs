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
using System.Collections.Generic;
using System.Linq;
using System.Windows.Markup;
namespace GaugeLib
{
    public class LinearScale:Scale
    {
        List<Rectangle> ranges=new List<Rectangle>();
        Rectangle def = new Rectangle();

        #region dependency properties

        public LinearTickPlacement TickPlacement
        {
            get { return (LinearTickPlacement)GetValue(TickPlacementProperty); }
            set { SetValue(TickPlacementProperty, value); }
        }
        public static readonly DependencyProperty TickPlacementProperty =
            DependencyProperty.Register("TickPlacement", typeof(LinearTickPlacement), typeof(LinearScale), new PropertyMetadata(LinearTickPlacement.TopLeft, TickPlacementPropertyChanged));

        public Orientation Orientation
        {
            get { return (Orientation)GetValue(OrientationProperty); }
            set { SetValue(OrientationProperty, value); }
        }
        public static readonly DependencyProperty OrientationProperty =
            DependencyProperty.Register("Orientation", typeof(Orientation), typeof(LinearScale), new PropertyMetadata(Orientation.Horizontal, OrientationPropertyChanged));

        #endregion

        public LinearScale()
        {
            CreateRanges();
        }

        #region dependency properties handlers
        private static void TickPlacementPropertyChanged(DependencyObject o, DependencyPropertyChangedEventArgs e)
        {
            //- redraw the ticks, labels, ranges and indicators            
            LinearScale scale = o as LinearScale;
            if (scale != null)
            {
                scale.RefreshLabels();
                scale.RefreshTicks();
                scale.RefreshIndicators();
            }
        }
        private static void OrientationPropertyChanged(DependencyObject o, DependencyPropertyChangedEventArgs e)
        {
            //- redraw the ticks, labels, ranges and indicators
            LinearScale scale = o as LinearScale;
            if (scale != null)
            {
                scale.RefreshLabels();
                scale.RefreshTicks();
                scale.RefreshIndicators();
            }
        }
        #endregion

        #region private helper functions
        private double GetSegmentOffset(double length, double value)
        {
            double offset = length * (value - Minimum) / (Maximum - Minimum);
            return offset;
        }
        #endregion

        #region overrides
        protected override Size MeasureOverride(Size availableSize)
        {
            foreach (Tick label in GetLabels())
                label.Measure(availableSize);
            foreach (Tick tick in GetTicks())
                tick.Measure(availableSize);
            foreach (Rectangle rect in ranges)
                rect.Measure(availableSize);
            
            if(Indicators!=null)
                foreach (UIElement ind in Indicators)
                    ind.Measure(availableSize);

            double width = 0;
            double height = 0;
            double lblMax=0, tickMax=0, indMax=0;
            if (Orientation == Orientation.Horizontal)
            {
                lblMax = GetLabels().Max(p => p.DesiredSize.Height);
                tickMax = GetTicks().Max(p => p.DesiredSize.Height);
                if (Indicators != null && Indicators.Count > 0)
                    indMax = Indicators.OfType<Indicator>().Max(p => p.DesiredSize.Height);
                height = 3 + lblMax + tickMax + indMax;
                if(!double.IsInfinity(availableSize.Width))
                    width = availableSize.Width;
            }
            else
            {
                lblMax = GetLabels().Max(p => p.DesiredSize.Width);
                tickMax = GetTicks().Max(p => p.DesiredSize.Width);
                if (Indicators != null && Indicators.Count > 0)
                    indMax = Indicators.OfType<Indicator>().Max(p => p.DesiredSize.Width);
                width = 3 + lblMax + tickMax + indMax;
                if (!double.IsInfinity(availableSize.Height))
                    height = availableSize.Height;
            }

            return new Size(width, height);
        }

        protected override void ArrangeTicks(Size finalSize)
        {
            var ticks = GetTicks();
            
            foreach (Tick tick in ticks)
            {
                if (Orientation == Orientation.Horizontal)
                {
                    double maxLength = ticks.Max(p => p.DesiredSize.Height);
                    double offset = GetSegmentOffset(finalSize.Width, tick.Value);
                    if (TickPlacement == LinearTickPlacement.TopLeft)
                    {
                        double yOff=GetLabels().Max(p=>p.DesiredSize.Height)+1;
                        tick.Arrange(new Rect(new Point(offset - tick.DesiredSize.Width / 2, yOff + maxLength - tick.DesiredSize.Height), tick.DesiredSize));
                    }
                    else
                    {
                        double yOff = finalSize.Height - maxLength - GetLabels().Max(p => p.DesiredSize.Height) - 2;
                        tick.Arrange(new Rect(new Point(offset - tick.DesiredSize.Width / 2, yOff), tick.DesiredSize));
                    }
                }
                else
                {
                    double maxLength = ticks.Max(p => p.DesiredSize.Width) + GetLabels().Max(p => p.DesiredSize.Width) + 2;
                    double offset = GetSegmentOffset(finalSize.Height, tick.Value);
                    if (TickPlacement == LinearTickPlacement.TopLeft)
                    {
                        tick.Arrange(new Rect(new Point(maxLength - tick.DesiredSize.Width,finalSize.Height- offset - tick.DesiredSize.Height / 2), tick.DesiredSize));
                    }
                    else
                    {
                        tick.Arrange(new Rect(new Point(finalSize.Width - maxLength,finalSize.Height- offset - tick.DesiredSize.Height / 2), tick.DesiredSize));
                    }
                }
            }
        }

        protected override void ArrangeLabels(Size finalSize)
        {
            var labels = GetLabels();
            double maxLength = labels.Max(p => p.DesiredSize.Width)+1;
            foreach (Tick tick in labels)
            {
                if (Orientation == Orientation.Horizontal)
                {
                    double offset = GetSegmentOffset(finalSize.Width, tick.Value);
                    if (TickPlacement == LinearTickPlacement.TopLeft)
                    {                        
                        tick.Arrange(new Rect(new Point(offset-tick.DesiredSize.Width/2, 1), tick.DesiredSize));
                    }
                    else
                    {
                        tick.Arrange(new Rect(new Point(offset - tick.DesiredSize.Width / 2, finalSize.Height - tick.DesiredSize.Height - 1), tick.DesiredSize));
                    }
                }
                else
                {
                    double offset = GetSegmentOffset(finalSize.Height, tick.Value);
                    if (TickPlacement == LinearTickPlacement.TopLeft)
                    {
                        tick.Arrange(new Rect(new Point(maxLength - tick.DesiredSize.Width,finalSize.Height - offset - tick.DesiredSize.Height / 2), tick.DesiredSize));
                    }
                    else
                    {
                        tick.Arrange(new Rect(new Point(finalSize.Width - maxLength,finalSize.Height - offset - tick.DesiredSize.Height / 2), tick.DesiredSize));
                    }
                }
            }
        }

        protected override void ArrangeRanges(Size finalSize)
        {
            if (UseDefaultRange)
            {
                if (Orientation == Orientation.Horizontal)
                {
                    double yOff;
                    if (TickPlacement == LinearTickPlacement.TopLeft)
                    {
                        yOff = GetLabels().Max(p => p.DesiredSize.Height) + GetTicks().Max(p => p.DesiredSize.Height) + 1;
                        def.Arrange(new Rect(new Point(0, yOff), new Size(finalSize.Width, RangeThickness)));
                    }
                    else
                    {
                        yOff = GetLabels().Max(p => p.DesiredSize.Height) + GetTicks().Max(p => p.DesiredSize.Height) + 3;
                        def.Arrange(new Rect(new Point(0, finalSize.Height - yOff - RangeThickness), new Size(finalSize.Width, RangeThickness)));
                    }

                }
                else
                {
                    double off = GetLabels().Max(p => p.DesiredSize.Width) + GetTicks().Max(p => p.DesiredSize.Width) + 2;
                    if (TickPlacement == LinearTickPlacement.TopLeft)
                    {
                        def.Arrange(new Rect(new Point(off, 0), new Size(RangeThickness, finalSize.Height)));
                    }
                    else
                    {
                        def.Arrange(new Rect(new Point(finalSize.Width - off - RangeThickness, 0), new Size(RangeThickness, finalSize.Height)));
                    }
                }
            }
            double posOffset = 0;
            var rng = Ranges.OrderBy(p => p.Offset).ToList();
            for (int i = 0; i < ranges.Count; i++)
            {
                Rectangle rect = ranges[i];
                rect.Fill = new SolidColorBrush(rng[i].Color);

                if (Orientation == Orientation.Horizontal)
                {
                    double yOff;
                    if (TickPlacement == LinearTickPlacement.TopLeft)
                    {
                        yOff = GetLabels().Max(p => p.DesiredSize.Height) + GetTicks().Max(p => p.DesiredSize.Height) + 1;
                        Size sz=new Size(GetSegmentOffset(finalSize.Width, rng[i].Offset) - posOffset, RangeThickness);
                        rect.Arrange(new Rect(new Point(posOffset, yOff), sz));
                    }
                    else
                    {
                        yOff = GetLabels().Max(p => p.DesiredSize.Height) + GetTicks().Max(p => p.DesiredSize.Height) + 3;
                        rect.Arrange(new Rect(new Point(posOffset, finalSize.Height - yOff - RangeThickness), new Size(GetSegmentOffset(finalSize.Width, rng[i].Offset) - posOffset, RangeThickness)));
                    }
                    posOffset = GetSegmentOffset(finalSize.Width, rng[i].Offset);
                }
                else
                {
                    double off = GetLabels().Max(p => p.DesiredSize.Width) + GetTicks().Max(p => p.DesiredSize.Width) + 2;
                    double segLength = GetSegmentOffset(finalSize.Height, rng[i].Offset);
                    if (TickPlacement == LinearTickPlacement.TopLeft)
                    {
                        rect.Arrange(new Rect(new Point(off, finalSize.Height - segLength), new Size(RangeThickness, segLength - posOffset)));
                    }
                    else
                    {
                        rect.Arrange(new Rect(new Point(finalSize.Width - off - RangeThickness, finalSize.Height - segLength), new Size(RangeThickness, segLength - posOffset)));
                    }
                    posOffset = segLength;
                }

            }
        }

        protected override DataTemplate CreateDataTemplate(TickType tType, double val)
        {
            String strBrush = DefaultTickColor.ToString();
            if (UseRangeColorsForTicks)
            {//change the brush according to the range color
                strBrush = GetRangeColorForValue(val).ToString();
            }

            double width = 2, height = 5, width2 = 2, height2 = 10;
            if (Orientation == Orientation.Vertical)
            {
                width = 5; height = 2; width2 = 10; height2 = 2;
            }
            
            string template = "";
            if (tType == TickType.Label)
                template = "<TextBlock Text=\"{Binding}\" Foreground=\"" + strBrush + "\" FontSize=\"13\" FontWeight=\"Bold\" />";
            else if (tType == TickType.Minor)
                template = "<Rectangle Fill=\""+strBrush+"\" Width=\"" + width + "\" Height=\"" + height + "\" />";
            else
                template = "<Rectangle Fill=\""+strBrush+"\" Width=\"" + width2 + "\" Height=\"" + height2 + "\" />";

            string xaml =
                @"<DataTemplate
        xmlns=""http://schemas.microsoft.com/winfx/2006/xaml/presentation""
        xmlns:x=""http://schemas.microsoft.com/winfx/2006/xaml"">" + template + @"</DataTemplate>";

            DataTemplate dt = (DataTemplate)XamlReader.Load(xaml);
            return dt;
        }
        
        protected override void RefreshRanges()
        {
            ClearRanges();
            CreateRanges();
        }
        protected override void CreateRanges()
        {
            //insert the default range
            if (UseDefaultRange)
            {
                def.Fill = new SolidColorBrush(DefaultRangeColor);
                if (!Children.Contains(def))
                    Children.Add(def);
            }
            if (Ranges == null)
                return;
            //it is presumed that the ranges are ordered
            foreach (GaugeRange r in Ranges)
            {
                Rectangle rect = new Rectangle();
                rect.Fill = new SolidColorBrush(r.Color);
                ranges.Add(rect);
                Children.Add(rect);
            }

        }
        protected override void ClearRanges()
        {
            //remove the default range
            if (UseDefaultRange)
            {
                Children.Remove(def);
            }
            if (Ranges == null)
                return;
            for (int i = 0; i < ranges.Count; i++)
            {
                Children.Remove(ranges[i]);
            }
            ranges.Clear();
        }
        
        #endregion
        
        internal Point GetIndicatorOffset(Indicator ind)
        {//get's the offset at which the indicator is placed inside the owner
            Point pos = new Point();
            if (Orientation == Orientation.Horizontal)
            {

                if (TickPlacement == LinearTickPlacement.TopLeft)
                {
                    pos.X = 0;
                    pos.Y = GetLabels().Max(p => p.DesiredSize.Height) + GetTicks().Max(p => p.DesiredSize.Height) + RangeThickness + 3;
                }
                else
                {
                    pos.X = 0;
                    pos.Y = ActualHeight - ind.DesiredSize.Height - (GetLabels().Max(p => p.DesiredSize.Height) + GetTicks().Max(p => p.DesiredSize.Height) + RangeThickness + 5);
                }
            }
            else
            {
                if (TickPlacement == LinearTickPlacement.TopLeft)
                {
                    pos.X = GetLabels().Max(p => p.DesiredSize.Width) + GetTicks().Max(p => p.DesiredSize.Width) + RangeThickness + 4;
                    pos.Y = ActualHeight - ind.DesiredSize.Height;
                }
                else
                {
                    pos.X = ActualWidth - ind.DesiredSize.Width - (GetLabels().Max(p => p.DesiredSize.Width) + GetTicks().Max(p => p.DesiredSize.Width) + RangeThickness + 4);
                    pos.Y = ActualHeight - ind.DesiredSize.Height;
                }
            }
            return pos;
        }
    }
}
