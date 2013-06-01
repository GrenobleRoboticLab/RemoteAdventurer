using System.Windows.Controls;
using System.Windows;
using System.Collections.ObjectModel;
using System.Windows.Markup;
using System.Collections.Generic;
using System;
using System.Diagnostics;
using System.Windows.Media;
using System.Linq;
namespace GaugeLib
{
    [ContentProperty("Indicators")]
    public abstract class Scale:Panel
    {
        #region private fields
        List<Tick> labels=new List<Tick>();
        List<Tick> ticks=new List<Tick>();
        Canvas indicatorContainer;
        #endregion

        public Scale()
        {
            //maybe add containers for ranges as well
            indicatorContainer = new Canvas();
            Children.Add(indicatorContainer);
            CreateLabels();
            CreateTicks();
        }

        public UIElementCollection Indicators
        {
            get { return indicatorContainer.Children; }
        }

        #region dependency properties

        public double Minimum
        {
            get { return (double)GetValue(MinimumProperty); }
            set { SetValue(MinimumProperty, value); }
        }
        public static readonly DependencyProperty MinimumProperty =
            DependencyProperty.Register("Minimum", typeof(double), typeof(Scale), new PropertyMetadata(0.0, MinimumPropertyChanged));

        public double Maximum
        {
            get { return (double)GetValue(MaximumProperty); }
            set { SetValue(MaximumProperty, value); }
        }
        public static readonly DependencyProperty MaximumProperty =
            DependencyProperty.Register("Maximum", typeof(double), typeof(Scale), new PropertyMetadata(100.0, MaximumPropertyChanged));

        #region tick related properties

        public int MinorTickStep
        {
            get { return (int)GetValue(MinorTickStepProperty); }
            set { SetValue(MinorTickStepProperty, value); }
        }
        public static readonly DependencyProperty MinorTickStepProperty =
            DependencyProperty.Register("MinorTickStep", typeof(int), typeof(Scale), new PropertyMetadata(5, TickStepPropertyChanged));

        public int MajorTickStep
        {
            get { return (int)GetValue(MajorTickStepProperty); }
            set { SetValue(MajorTickStepProperty, value); }
        }
        public static readonly DependencyProperty MajorTickStepProperty =
            DependencyProperty.Register("MajorTickStep", typeof(int), typeof(Scale), new PropertyMetadata(10, TickStepPropertyChanged));

        public DataTemplate MinorTickTemplate
        {
            get { return (DataTemplate)GetValue(MinorTickTemplateProperty); }
            set { SetValue(MinorTickTemplateProperty, value); }
        }
        public static readonly DependencyProperty MinorTickTemplateProperty =
            DependencyProperty.Register("MinorTickTemplate", typeof(DataTemplate), typeof(Scale), new PropertyMetadata(null, TickTemplatesChanged));

        public DataTemplate MajorTickTemplate
        {
            get { return (DataTemplate)GetValue(MajorTickTemplateProperty); }
            set { SetValue(MajorTickTemplateProperty, value); }
        }
        public static readonly DependencyProperty MajorTickTemplateProperty =
            DependencyProperty.Register("MajorTickTemplate", typeof(DataTemplate), typeof(Scale), new PropertyMetadata(null, TickTemplatesChanged));

        public DataTemplate LabelTemplate
        {
            get { return (DataTemplate)GetValue(LabelTemplateProperty); }
            set { SetValue(LabelTemplateProperty, value); }
        }
        public static readonly DependencyProperty LabelTemplateProperty =
            DependencyProperty.Register("LabelTemplate", typeof(DataTemplate), typeof(Scale), new PropertyMetadata(null, TickTemplatesChanged));

        public Color DefaultTickColor
        {
            get { return (Color)GetValue(DefaultTickColorProperty); }
            set { SetValue(DefaultTickColorProperty, value); }
        }
        public static readonly DependencyProperty DefaultTickColorProperty =
            DependencyProperty.Register("DefaultTickColor", typeof(Color), typeof(Scale), new PropertyMetadata(Colors.White, TickTemplatesChanged));

        
        #endregion

        #region range related properties

        public ObservableCollection<GaugeRange> Ranges
        {
            get { return (ObservableCollection<GaugeRange>)GetValue(RangesProperty); }
            set { SetValue(RangesProperty, value); }
        }
        public static readonly DependencyProperty RangesProperty =
            DependencyProperty.Register("Ranges", typeof(ObservableCollection<GaugeRange>), typeof(Scale), new PropertyMetadata(new ObservableCollection<GaugeRange>(), RangesPropertyChanged));

        public int RangeThickness
        {
            get { return (int)GetValue(RangeThicknessProperty); }
            set { SetValue(RangeThicknessProperty, value); }
        }
        public static readonly DependencyProperty RangeThicknessProperty =
            DependencyProperty.Register("RangeThickness", typeof(int), typeof(Scale), new PropertyMetadata(1, RangeRelatedPropertyChanged));

        public bool UseDefaultRange
        {
            get { return (bool)GetValue(UseDefaultRangeProperty); }
            set { SetValue(UseDefaultRangeProperty, value); }
        }
        public static readonly DependencyProperty UseDefaultRangeProperty =
            DependencyProperty.Register("UseDefaultRange", typeof(bool), typeof(Scale), new PropertyMetadata(true, RangeRelatedPropertyChanged));


        public Color DefaultRangeColor
        {
            get { return (Color)GetValue(DefaultRangeColorProperty); }
            set { SetValue(DefaultRangeColorProperty, value); }
        }
        public static readonly DependencyProperty DefaultRangeColorProperty =
            DependencyProperty.Register("DefaultRangeColor", typeof(Color), typeof(Scale), new PropertyMetadata(Colors.White,RangeRelatedPropertyChanged));

        
        public bool UseRangeColorsForTicks
        {
            get { return (bool)GetValue(UseRangeColorsForTicksProperty); }
            set { SetValue(UseRangeColorsForTicksProperty, value); }
        }
        public static readonly DependencyProperty UseRangeColorsForTicksProperty =
            DependencyProperty.Register("UseRangeColorsForTicks", typeof(bool), typeof(Scale), new PropertyMetadata(false, TickTemplatesChanged));

        #endregion

        #endregion

        #region dependency properties handlers
        private static void MinimumPropertyChanged(DependencyObject o, DependencyPropertyChangedEventArgs e)
        {
            Scale scale = o as Scale;
            if (scale != null)
            {
                scale.RefreshLabels();
                scale.RefreshTicks();
                scale.RefreshRanges();
                scale.RefreshIndicators();
            }
        }
        private static void MaximumPropertyChanged(DependencyObject o, DependencyPropertyChangedEventArgs e)
        {
            Scale scale = o as Scale;
            if (scale != null)
            {
                scale.RefreshLabels();
                scale.RefreshTicks();
                scale.RefreshRanges();
                scale.RefreshIndicators();
            }
        }
        private static void TickStepPropertyChanged(DependencyObject o, DependencyPropertyChangedEventArgs e)
        {
            //- update the ticks and labels
            Scale scale = o as Scale;
            if (scale != null)
            {
                if (scale.MinorTickStep == 0) scale.MinorTickStep = 1;
                if (scale.MajorTickStep == 0) scale.MajorTickStep = 1;
                scale.RefreshLabels();
                scale.RefreshTicks();
            }
        }
        private static void TickTemplatesChanged(DependencyObject o, DependencyPropertyChangedEventArgs e)
        {
            //- update the ticks and labels
            Scale scale = o as Scale;
            if (scale != null)
            {
                scale.RefreshLabels();
                scale.RefreshTicks();
                scale.RefreshIndicators();
            }
        }
        private static void RangesPropertyChanged(DependencyObject o, DependencyPropertyChangedEventArgs e)
        {
            //- update the ranges
            //- update the ticks and labels
            Scale scale = o as Scale;
            if (scale != null)
            {
                if (e.OldValue != null)
                {
                    ((ObservableCollection<GaugeRange>)e.OldValue).CollectionChanged -= scale.Ranges_CollectionChanged;
                }

                if (e.NewValue != null)
                {
                    ObservableCollection<GaugeRange> col = e.NewValue as ObservableCollection<GaugeRange>;
                    col.CollectionChanged += scale.Ranges_CollectionChanged;
                    scale.RefreshRanges();
                    scale.RefreshTicks();
                    scale.RefreshLabels();
                }
            }
        }
        void Ranges_CollectionChanged(object sender, System.Collections.Specialized.NotifyCollectionChangedEventArgs e)
        {
            RefreshRanges();
            RefreshLabels();
            RefreshTicks();
        }
        private static void RangeRelatedPropertyChanged(DependencyObject o, DependencyPropertyChangedEventArgs e)
        {
            //- update the ranges
            //- update the ticks and labels
            //update the indicators
            Scale scale = o as Scale;
            if (scale != null)
            {
                scale.RefreshRanges();
                scale.RefreshLabels();
                scale.RefreshTicks();
                scale.RefreshIndicators();
            }
        }
        #endregion

        #region abstract methods
        protected abstract void ArrangeTicks(Size finalSize);
        protected abstract void ArrangeLabels(Size finalSize);
        protected abstract void ArrangeRanges(Size finalSize);
        //range shape depends on the scale type so the ranges should
        //be created in the derived types
        protected abstract void CreateRanges();
        protected abstract void ClearRanges();
        #endregion

        #region overiddes
        protected override Size ArrangeOverride(Size finalSize)
        {
            // TODO : Commented to avoid a bug
            // Debug.WriteLine("entered Scale.ArrangeOverride");
            //arrange the children. since this is dependent of the scale type
            //the arranging will be done in the derived classes
            ArrangeLabels(finalSize);
            ArrangeTicks(finalSize);
            ArrangeRanges(finalSize);
            //arrange the indicator container to occupy the entire available
            //size. this will also call arrange on the children
            //indicatorContainer.Arrange(new Rect(new Point(), finalSize));
            //at the end just return the final size
            ArrangeIndicators(finalSize);
            return finalSize;
        }
        #endregion
        private void ArrangeIndicators(Size finalSize)
        {
            foreach (Indicator ind in Indicators)
                ind.Arrange(finalSize);
        }

        #region private helper methods
        private void CreateLabels()
        {
            double max = Maximum;
            double min = Minimum;
            for (double v = min; v <= max; v += MajorTickStep)
            {
                Tick tick = new Tick() { Value = v, TickType = TickType.Label };
                labels.Add(tick);
                //also set the content and template for the label
                tick.ContentTemplate = GetTickTemplate(TickType.Label, v);
                tick.Content = v;
                Children.Insert(0, tick);
            }
        }
        private void CreateTicks()
        {
            double max = Maximum;
            double min = Minimum;
            int num = 0;//the tick index
            double val = min;//the value of the tick
            while (val <= max)
            {
                DataTemplate template = null;
                Tick tick = new Tick();
                tick.Value = val;
                if (num % MinorTickStep == 0)
                {
                    tick.TickType = TickType.Minor;
                    template = GetTickTemplate(TickType.Minor, val);
                }
                if (num % MajorTickStep == 0)
                {
                    tick.TickType = TickType.Major;
                    template = GetTickTemplate(TickType.Major, val);
                }
                tick.ContentTemplate = template;
                tick.Content = val;
                ticks.Add(tick);
                Children.Insert(0, tick);

                val += MinorTickStep;
                num += MinorTickStep;
            }
        }
        private void ClearLabels()
        {
            for (int i = 0; i < labels.Count; i++)
            {
                Children.Remove(labels[i]);
            }
            labels.Clear();
        }
        private void ClearTicks()
        {
            for (int i = 0; i < ticks.Count; i++)
            {
                Children.Remove(ticks[i]);
            }
            ticks.Clear();
        }
        //used to supply a default template for the ticks and labels
        //override this in the derived scale classes
        protected virtual DataTemplate CreateDataTemplate(TickType tType, double val)
        {
            String strBrush = DefaultTickColor.ToString();
            if (UseRangeColorsForTicks)
            {//change the brush according to the range color
                strBrush = GetRangeColorForValue(val).ToString();
            }
            string template = "";
            if (tType == TickType.Label)
                template = "<TextBlock Text=\"{Binding}\" Foreground=\""+strBrush+"\" FontSize=\"13\" FontWeight=\"Bold\" />";
            else if (tType == TickType.Minor)
                template = "<Rectangle Fill=\""+strBrush+"\" Width=\"2\" Height=\"5\" />";
            else
                template = "<Rectangle Fill=\"" + strBrush + "\" Width=\"2\" Height=\"10\" />";

            string xaml =
                @"<DataTemplate
        xmlns=""http://schemas.microsoft.com/winfx/2006/xaml/presentation""
        xmlns:x=""http://schemas.microsoft.com/winfx/2006/xaml"">"+template+@"</DataTemplate>";

            DataTemplate dt = (DataTemplate)XamlReader.Load(xaml);
            return dt;
        }
        protected Color GetRangeColorForValue(double val)
        {
            //the value can be in more than 1 range since if it is in a range and 
            //we might have another range with a bigger offset. ex:
            // a value of 10 will be in ranges with offsets of 15, 20, etc
            //this is why i order the ranges and return the first match
            var rngs = Ranges.OrderBy(p => p.Offset).ToList();
            for (int i = 0; i < rngs.Count; i++)
            {
                if (val <= rngs[i].Offset)
                    return rngs[i].Color;
            }
            return DefaultRangeColor;
        }
        private DataTemplate GetTickTemplate(TickType tType, double val)
        {
            //after adding template properties also check that those arent null
            if (tType == TickType.Label)
            {
                if (LabelTemplate != null)
                    return LabelTemplate;
                return CreateDataTemplate(tType, val);
            }
            else if (tType == TickType.Minor)
            {
                if (MinorTickTemplate != null)
                    return MinorTickTemplate;
                return CreateDataTemplate(tType, val);
            }
            else
            {
                if (MajorTickTemplate != null)
                    return MajorTickTemplate;
                return CreateDataTemplate(tType, val);
            }
        }

        protected List<Tick> GetTicks()
        {
            return ticks;
        }
        protected List<Tick> GetLabels()
        {
            return labels;
        }

        protected void RefreshTicks()
        {
            ClearTicks();
            CreateTicks();
        }
        protected void RefreshLabels()
        {
            ClearLabels();
            CreateLabels();
        }
        protected void RefreshIndicators()
        {
            foreach (Indicator ind in Indicators)
            {
                ind.InvalidateMeasure();
                ind.InvalidateArrange();
            }
        }
        protected abstract void RefreshRanges();
        
        #endregion
    }
}
