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
using System.Windows.Markup;

namespace GaugeLib
{
    [TemplatePart(Name = "PART_Marker", Type = typeof(ContentPresenter))]
    public class MarkerIndicator:Indicator
    {
        ContentPresenter marker;
        public MarkerIndicator()
        {
            DefaultStyleKey = typeof(MarkerIndicator);
        }

        #region dependency properties

        public DataTemplate MarkerTemplate
        {
            get { return (DataTemplate)GetValue(MarkerTemplateProperty); }
            set { SetValue(MarkerTemplateProperty, value); }
        }

        // Using a DependencyProperty as the backing store for MarkerTemplate.  This enables animation, styling, binding, etc...
        public static readonly DependencyProperty MarkerTemplateProperty =
            DependencyProperty.Register("MarkerTemplate", typeof(DataTemplate), typeof(MarkerIndicator), new PropertyMetadata(null, MarkerTemplatePropertyChanged));

        private static void MarkerTemplatePropertyChanged(DependencyObject o, DependencyPropertyChangedEventArgs e)
        {
            //update the indicator
            MarkerIndicator ind = o as MarkerIndicator;
            if (ind != null && ind.marker!=null)
            {
                ind.marker.ContentTemplate = ind.MarkerTemplate;
            }
        }
        #endregion

        #region overrides
        public override void OnApplyTemplate()
        {
            base.OnApplyTemplate();
            marker = GetTemplateChild("PART_Marker") as ContentPresenter;
        }
        public override void Arrange(Size finalSize)
        {
            base.Arrange(DesiredSize);
            //call method to arrange the marker
            SetIndicatorTransforms();
            PositionMarker();
        }

        protected override void OnValueChanged(double newVal, double oldVal)
        {
            PositionMarker();    
        }
        #endregion

        #region private methods
        //sets the render transform of the indicator
        private void SetIndicatorTransforms()
        {
            if (RenderTransform is MatrixTransform)
            {
                TransformGroup tg = new TransformGroup();
                TranslateTransform tt = new TranslateTransform();
                RotateTransform rt = new RotateTransform();

                tg.Children.Add(rt);
                tg.Children.Add(tt);

                this.RenderTransformOrigin = new Point(0.5, 0.5);
                this.RenderTransform = tg;
            }
        }

        private void PositionMarker()
        {
            if (Owner == null)
                return;
            if (Owner is RadialScale)
            {
                RadialScale rs = (RadialScale)Owner;
                //get the angle based on the value
                double angle = rs.GetAngleFromValue(Value);
                if (rs.SweepDirection == SweepDirection.Counterclockwise)
                {
                    angle = -angle;
                }
                //rotate the marker by angle
                TransformGroup tg = RenderTransform as TransformGroup;
                if (tg != null)
                {
                    RotateTransform rt = tg.Children[0] as RotateTransform;
                    if (rt != null)
                    {
                        rt.Angle = angle;
                    }
                }
                //position the marker based on the radius
                Point offset = rs.GetIndicatorOffset();
                double rad = rs.GetIndicatorRadius();

                //position the marker
                double px = offset.X + (rad - DesiredSize.Height / 2) * Math.Sin(angle * Math.PI / 180);
                double py = offset.Y - (rad - DesiredSize.Height / 2) * Math.Cos(angle * Math.PI / 180);
                px -= DesiredSize.Width / 2;
                py -= DesiredSize.Height / 2;
                if (tg != null)
                {
                    TranslateTransform tt = tg.Children[1] as TranslateTransform;
                    if (tt != null)
                    {
                        tt.X = px;
                        tt.Y = py;
                    }
                }
            }
            else
            {
                LinearScale ls = Owner as LinearScale;
                Point offset = ls.GetIndicatorOffset(this);
                //the getIndicatorOffset returns only one correct dimension
                //for marker indicators the other dimension will have to be calculated again
                if (ls.Orientation == Orientation.Horizontal)
                {
                    offset.X = ls.ActualWidth * (Value - ls.Minimum) / (ls.Maximum - ls.Minimum) - DesiredSize.Width / 2;
                }
                else
                {
                    offset.Y = ls.ActualHeight - ls.ActualHeight * (Value - ls.Minimum) / (ls.Maximum - ls.Minimum) - DesiredSize.Height / 2;
                }
                TransformGroup tg = RenderTransform as TransformGroup;
                if (tg != null)
                {
                    TranslateTransform tt = tg.Children[1] as TranslateTransform;
                    if (tt != null)
                    {
                        tt.X = offset.X;
                        tt.Y = offset.Y;
                    }
                }
            }
        }
        
        #endregion
    }
}
