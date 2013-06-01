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
    public abstract class Indicator:Control
    {
        #region fields
        private Scale owner;
        #endregion

        #region properties
        public Scale Owner
        {
            get
            {
                return this.owner;
            }
            internal set
            {
                if (this.owner != value)
                {
                    this.owner = value;
                    UpdateIndicator(owner);
                }
            }
        }
        #endregion

        #region dependency properties
        public double Value
        {
            get { return (double)GetValue(ValueProperty); }
            set { SetValue(ValueProperty, value); }
        }
        public static readonly DependencyProperty ValueProperty =
            DependencyProperty.Register("Value", typeof(double), typeof(Indicator), new PropertyMetadata(0.0, ValuePropertyChanged));

        #endregion

        #region dp handlers
        private static void ValuePropertyChanged(DependencyObject o, DependencyPropertyChangedEventArgs e)
        {
            Indicator ind = o as Indicator;
            if (ind != null)
            {
                if (ind.Owner!=null && ind.Value >= ind.Owner.Minimum && ind.Value <= ind.Owner.Maximum)
                    ind.OnValueChanged((double)e.NewValue, (double)e.OldValue);                
            }
        }
        protected virtual void OnValueChanged(double newVal, double oldVal) { }
        #endregion

        #region abstract methods
        protected virtual void UpdateIndicatorOverride(Scale owner) { }
        #endregion

        #region private helper methods
        private void UpdateIndicator(Scale owner)
        {
            if (owner != null)
            {
                if (Value < owner.Minimum)
                    Value = owner.Minimum;
                if (Value > owner.Maximum)
                    Value = owner.Maximum;
            }
            UpdateIndicatorOverride(owner);
        }
        
        #endregion
        //this wasn't virtual in the previous version
        public virtual void Arrange(Size finalSize)
        {
            base.Arrange(new Rect(new Point(), finalSize));
        }
        #region overrides
        protected override Size MeasureOverride(Size availableSize)
        {
            //the main purpose of this override is to set the owner for the 
            //indicator. The actual measuring calculation will be done in 
            //the derived classes
            DependencyObject parent = base.Parent;
            while (parent != null)
            {
                Scale scale = parent as Scale;
                if (scale != null)
                {
                    this.Owner = scale;
                    break;
                }
                FrameworkElement el = parent as FrameworkElement;
                if (el != null)
                {
                    parent = el.Parent;
                }
            }
            return base.MeasureOverride(availableSize);
        }
        #endregion

    }
}
