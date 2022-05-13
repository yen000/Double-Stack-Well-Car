using System;
using System.Collections.Generic;
using System.Text;

namespace Double_Stack_Well_Car
{
    class Function
    {
        public static double[] initial_array(double[] obj_array, double value)
        {

            double[] result = new double[obj_array.Length];

            for (int i = 0; i < obj_array.Length; i++)
            {
                result[i] = value;
            }

            return result;

        }

        public static double[] array_to_double(string[] obj_array)
        {

            double[] result = new double[obj_array.Length];

            for (int i = 0; i < obj_array.Length; i++)
            {
                result[i] = double.Parse(obj_array[i]);
            }

            return result;

        }

        public static List<double> get_hub_sets(List<List<double>> list1, List<List<double>> list2, List<List<double>> list3)
        {

            List<double> result = new List<double>();

            for (int i = 0; i < list1.Count; i++)
            {
                if (!whether_in_list(result, list1[i][1]))
                {
                    result.Add(list1[i][1]);
                }
            }

            for (int i = 0; i < list2.Count; i++)
            {
                if (!whether_in_list(result, list2[i][1]))
                {
                    result.Add(list2[i][1]);
                }
            }

            for (int i = 0; i < list3.Count; i++)
            {
                if (!whether_in_list(result, list3[i][1]))
                {
                    result.Add(list3[i][1]);
                }
            }

            return result;

        }

        public static bool whether_in_list(List<double> objective, double value)
        {

            bool result = false;

            for (int i = 0; i < objective.Count; i++)
            {
                if (objective[i] == value)
                {
                    result = true;
                }
            }

            return result;

        }

        public static int find_destination(double[][] objective, int car_index)
        {

            int result = -1;

            for (int i = 0; i < objective.Length; i++)
            {
                if (objective[i][car_index] == 1)
                {
                    result = i;
                }
            }


            return result;

        }
        public static List<double> container_amount(List<List<double>> container_list)
        {
            List<double> result = new List<double>();

            for (int i = 0; i <= container_list.Count - 1; i++)
            {
                result.Add(i + 1);
            }

            return result;
        }
        public static bool check_stage2(int car_amount, List<List<double>> w20l, List<List<double>> w20e, List<List<double>> w40, List<double> hub_set)
        {
            bool result = false;

            if (car_amount > 0)
            {
                if (w20l.Count >= 2 && same_hub_check(w20l, hub_set) == true)
                {
                    result = true;
                    goto output;
                }
                if (w20e.Count >= 2 && same_hub_check(w20e, hub_set) == true)
                {
                    result = true;
                    goto output;
                }
                if (w40.Count >= 1 && same_hub_check(w40, hub_set) == true)
                {
                    result = true;
                    goto output;
                }
            }
        output:
            return result;

        }
        public static bool same_hub_check(List<List<double>> container, List<double> hub_set)
        {
            bool result = false;
            int[] count = new int[hub_set.Count];

            for (int k = 0; k < hub_set.Count; k++)
            {
                for (int i = 0; i < container.Count; i++)
                {
                    if (container[i][1] == k + 1)
                    {
                        count[k]++;
                    }
                }
            }
            for (int j = 0; j < count.Length; j++)
            {
                if (count[j] > 1)
                {
                    result = true;
                    break;
                }
            }


            return result;
        }
        public static double utility_upper_bound(int car_amount, int car_amount_all, double utility_first_stage, List<List<double>> w20l, List<List<double>> w20e, List<List<double>> w40, List<double> hub_set)
        {
            double utility_upper_bound = utility_first_stage;
            int a = 0, b = 0, c = 0;
            int[] count_a = count_pair(w20l, hub_set);
            if (car_amount > 0)
            {
                for (int i = 0; i < count_a.Length; i++)
                {
                    if (count_a[i] >= 2)
                    {
                        a = (count_a[i] / 2);
                        Console.WriteLine("a " + a);
                        utility_upper_bound = utility_upper_bound + (a * 0.5 / car_amount_all);
                        car_amount--;
                        if (car_amount <= 0)
                            break;
                    }

                }
            }

            if (car_amount > 0)
            {
                int[] count_b = count_pair(w20e, hub_set);
                for (int i = 0; i < count_b.Length; i++)
                {
                    if (count_b[i] >= 2)
                    {
                        b = (count_b[i] / 2);
                        utility_upper_bound += b * 0.5 / car_amount_all;
                        car_amount--;
                        if (car_amount <= 0)
                            break;
                    }

                }
            }

            int[] count_c = count_pair(w40, hub_set);
            if (car_amount > 0)
            {
                for (int i = 0; i < count_c.Length; i++)
                {
                    if (count_c[i] >= 1)
                    {
                        c = (count_c[i]);
                        utility_upper_bound += c * 0.5 / car_amount_all;
                        car_amount--;
                        if (car_amount <= 0)
                            break;
                    }

                }
            }
            return utility_upper_bound;

        }
        public static int[] count_pair(List<List<double>> container, List<double> hub_set)
        {
            int[] count_pair = new int[hub_set.Count];

            for (int k = 0; k < hub_set.Count; k++)
            {
                for (int i = 0; i < container.Count; i++)
                {
                    if (container[i][1] == k + 1)
                    {
                        count_pair[k]++;
                    }
                }
            }
            return count_pair;

        }
    }
}
