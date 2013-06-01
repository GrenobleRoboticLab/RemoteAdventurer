using System;

namespace NXTRemote.Gauges
{
    [System.Windows.Markup.ContentProperty("Ranges")]
    public class RangeTemplateConverter : System.Windows.Data.IValueConverter
    {
        public System.Collections.ObjectModel.ObservableCollection<TemplateRange> Ranges { get; set; }
        public System.Windows.DataTemplate DefaultTemplate { get; set; }
        public RangeTemplateConverter()
        {
            Ranges = new System.Collections.ObjectModel.ObservableCollection<TemplateRange>();
        }
        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            double val = (double)value;
            for (int i = 0; i < Ranges.Count; i++)
            {
                if (Ranges[i].Minimum < val && val <= Ranges[i].Maximum)
                    return Ranges[i].Template;
            }
            return DefaultTemplate;
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }
}
