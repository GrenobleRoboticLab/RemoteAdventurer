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
    [TemplatePart(Name="PART_Needle", Type=typeof(Path))]
    public class NeedleIndicator:Indicator
    {
        Path needle;
        public NeedleIndicator()
        {
            DefaultStyleKey = typeof(NeedleIndicator);
        }
        public override void OnApplyTemplate()
        {
            base.OnApplyTemplate();
            needle = GetTemplateChild("PART_Needle") as Path;
        }
        protected override void UpdateIndicatorOverride(Scale owner)
        {
            base.UpdateIndicatorOverride(owner);
            SetIndicatorTransforms();
            RadialScale scale = owner as RadialScale;
            if(scale!=null)
            {
                SetIndicatorAngle(scale, Value);
            }
        }

        public override void Arrange(Size finalSize)
        {
            base.Arrange(DesiredSize);
            //arrange the indicator in the center
            SetIndicatorTransforms();
            RadialScale scale = Owner as RadialScale;
            if (scale != null)
            {
                Point center = scale.GetIndicatorOffset();
                TransformGroup tg = RenderTransform as TransformGroup;
                if (tg != null)
                {
                    //add a scale in order to make the needle to feet exactly inside the range
                    ScaleTransform st = tg.Children[0] as ScaleTransform;
                    double rad = scale.GetIndicatorRadius();
                    if (st != null && DesiredSize.Height != 0 && !double.IsInfinity(DesiredSize.Height) && rad > 0)
                    {
                        //factor is the radius devided by the height
                        double factor = rad / ( DesiredSize.Height);
                        st.ScaleX = factor;
                        st.ScaleY = factor;
                    }
                    TranslateTransform tt = tg.Children[2] as TranslateTransform;
                    if (tt != null)
                    {
                        tt.X = center.X - DesiredSize.Width / 2;
                        tt.Y = center.Y - DesiredSize.Height;
                    }
                }

            }
        }
        protected override void OnValueChanged(double newVal, double oldVal)
        {
            RadialScale scale = Owner as RadialScale;
            if (scale != null)
            {
                SetIndicatorAngle(scale, Value);
            }
        }
        //sets the transforms that will be used by the indicator
        private void SetIndicatorTransforms()
        {
            if (RenderTransform is MatrixTransform)
            {
                TransformGroup tg = new TransformGroup();
                TranslateTransform tt = new TranslateTransform();
                RotateTransform rt = new RotateTransform();
                ScaleTransform st = new ScaleTransform();

                tg.Children.Add(st);
                tg.Children.Add(rt);
                tg.Children.Add(tt);

                this.RenderTransformOrigin = new Point(0.5, 1);
                this.RenderTransform = tg;
            }
        }
        //sets the orientation of the indicator based on the scale and on the value
        private void SetIndicatorAngle(RadialScale scale, double value)
        {
            double angle = scale.GetAngleFromValue(Value);
            if (scale.SweepDirection == SweepDirection.Counterclockwise)
            {
                angle = -angle;
            }
            //rotate the needle
            TransformGroup tg = RenderTransform as TransformGroup;
            if (tg != null)
            {
                RotateTransform rt = tg.Children[1] as RotateTransform;
                if (rt != null)
                {
                    rt.Angle = angle;
                    // TODO : Commented to avoid a bug
                    // Debug.WriteLine("angle changed to " + angle);
                }
            }
        }
    }
}
