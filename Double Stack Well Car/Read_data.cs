using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using ILOG.Concert;
using ILOG.CPLEX;

namespace Double_Stack_Well_Car
{
    class Read_data
    {
        public static List<List<double>> w20l = new List<List<double>>();
        public static List<List<double>> w20e = new List<List<double>>();
        public static List<List<double>> w40 = new List<List<double>>();

        public static List<double> hub_set = new List<double>();

        public static double[] x_weights = new double[0];
        public static double[] v_weights = new double[0];

        public static double[] y_weights = new double[0];

        public static int hub_amount;
        public static double[] weight_limit, weight_tolerence_factor;
        public static int car_amount;
        public static int stack_amount = 2;

        public static void model(string file_name)
        {

            string file_path = file_name + "\\w20l.csv";

            if (file_path != "none")
            {
                StreamReader w20l_file = new StreamReader(file_path);

                string[] values = null;
                string data = w20l_file.ReadLine();

                while ((data = w20l_file.ReadLine()) != null)
                {
                    values = data.Split(',');
                    w20l.Add(new List<double> { double.Parse(values[1]), double.Parse(values[2]) });
                }
            }

            file_path = file_name + "\\w20e.csv";

            if (file_path != "none")
            {
                StreamReader w20e_file = new StreamReader(file_path);

                string[] values = null;
                string data = w20e_file.ReadLine();

                while ((data = w20e_file.ReadLine()) != null)
                {
                    values = data.Split(',');
                    w20e.Add(new List<double> { double.Parse(values[1]), double.Parse(values[2]) });
                }
            }

            file_path = file_name + "\\w40.csv";

            if (file_path != "none")
            {
                StreamReader w40_file = new StreamReader(file_path);

                string[] values = null;
                string data = w40_file.ReadLine();

                while ((data = w40_file.ReadLine()) != null)
                {
                    values = data.Split(',');
                    w40.Add(new List<double> { double.Parse(values[1]), double.Parse(values[2]) });
                }
            }

            List<double> hub_set = Function.get_hub_sets(w20l, w20e, w40);

            x_weights = new double[w20l.Count];

            for (int i = 0; i < w20l.Count; i++)
            {
                x_weights[i] = w20l[i][0];
            }

            v_weights = new double[w20e.Count];

            for (int i = 0; i < w20e.Count; i++)
            {
                v_weights[i] = w20e[i][0];
            }

            y_weights = new double[w40.Count];

            for (int i = 0; i < w40.Count; i++)
            {
                y_weights[i] = w40[i][0];
            }

            hub_amount = hub_set.Count;


            file_path = file_name + "\\car.csv";



            if (file_path.Length > 10)
            {

                StreamReader car_file = new StreamReader(file_path);
                List<List<double>> car_info = new List<List<double>>();
                string[] values = null;

                string car_data = car_file.ReadLine();

                while ((car_data = car_file.ReadLine()) != null)
                {
                    values = car_data.Split(',');
                    car_info.Add(new List<double> { double.Parse(values[1]), double.Parse(values[2]) });
                }

                car_amount = car_info.Count;

                weight_limit = new double[car_amount];
                weight_tolerence_factor = new double[car_amount];

                for (int i = 0; i < car_amount; i++)
                {
                    weight_limit[i] = car_info[i][0];
                    weight_tolerence_factor[i] = car_info[i][1];
                }

            }
            else
            {

                Console.Write("\n+---Set parameters---+\nSet the car's amount:\n> ");

                car_amount = int.Parse(Console.ReadLine());

                weight_limit = new double[car_amount];
                weight_tolerence_factor = new double[car_amount];

                Console.Write("\nSet the car's weight limit:\n> ");
                weight_limit = Function.initial_array(weight_limit, double.Parse(Console.ReadLine()));

                Console.Write("\nSet the car's tolerence factor of (upper_weight/lower_weight)\n> ");
                weight_tolerence_factor = Function.initial_array(weight_tolerence_factor, double.Parse(Console.ReadLine()));

            }


            string result_file_path = file_name + "\\original_result.csv";
        }
    }
}
