using System;

namespace NXTRemote.Gauges
{
    [System.Windows.Markup.ContentProperty("Ranges")]
    public class RangeColorConverter : System.Windows.Data.IValueConverter
    {
        public System.Collections.ObjectModel.ObservableCollection<ColorRange> Ranges { get; set; }
        public System.Windows.Media.Color DefaultColor { get; set; }
        public RangeColorConverter()
        {
            Ranges = new System.Collections.ObjectModel.ObservableCollection<ColorRange>();
            DefaultColor = System.Windows.Media.Colors.White;
        }
        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            double val = (double)value;
            for (int i = 0; i < Ranges.Count; i++)
            {
                if (Ranges[i].Minimum < val && val <= Ranges[i].Maximum)
                    return new System.Windows.Media.SolidColorBrush(Ranges[i].Color);
            }
            return new System.Windows.Media.SolidColorBrush(DefaultColor);
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }
}
