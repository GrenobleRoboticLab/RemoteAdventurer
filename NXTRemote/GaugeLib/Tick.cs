using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;

namespace GaugeLib
{
    public enum TickType { Minor, Major, Label }
    public class Tick:ContentPresenter
    {
        public double Value
        {
            get { return (double)GetValue(ValueProperty); }
            set { SetValue(ValueProperty, value); }
        }
        public static readonly DependencyProperty ValueProperty =
            DependencyProperty.Register("ValueProperty", typeof(double), typeof(Tick), new PropertyMetadata(0.0));

        public TickType TickType
        {
            get { return (TickType)GetValue(TickTypeProperty); }
            set { SetValue(TickTypeProperty, value); }
        }
        public static readonly DependencyProperty TickTypeProperty =
            DependencyProperty.Register("TickType", typeof(TickType), typeof(Tick), new PropertyMetadata(TickType.Minor));

    }
}
