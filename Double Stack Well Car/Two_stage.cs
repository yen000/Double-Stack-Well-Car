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
    class Two_stage
    {
        public static List<List<double>> w20l = new List<List<double>>();
        public static List<List<double>> w20e = new List<List<double>>();
        public static List<List<double>> w40 = new List<List<double>>();
        public static List<List<double>> w20l_second = new List<List<double>>();
        public static List<List<double>> w20e_second = new List<List<double>>();
        public static List<List<double>> w40_second = new List<List<double>>();

        public static List<List<double>> w20l_rest = new List<List<double>>();
        public static List<List<double>> w20e_rest = new List<List<double>>();
        public static List<List<double>> w40_rest = new List<List<double>>();

        public static bool check_enter_three_stage;
        public static int car_for_three_stage;
        public static double utility_first_stage;
        public static double total_time_two_stage;
        public static void model()
        {
            w20l = Read_data.w20l;
            w20e = Read_data.w20e;
            w40 = Read_data.w40;
            w20l_second = Read_data.w20l;
            w20e_second = Read_data.w20e;
            w40_second = Read_data.w40;

            int stack_amount = 2;

            List<double> hub_set = Function.get_hub_sets(w20l, w20e, w40);

            int hub_amount = hub_set.Count;
            int car_amount = Read_data.car_amount;

            int car_amount_all = Read_data.car_amount;

            #region first_stage
            Console.WriteLine("Two_stage");

            // input the information

            Console.Write("Please drag the w20l-file(.csv) in the console, 'none' if there's no data\n> ");

            double[] x_weights = new double[w20l.Count];

            for (int i = 0; i < w20l.Count; i++)
            {
                x_weights[i] = w20l[i][0];
            }

            double[] v_weights = new double[w20e.Count];

            for (int i = 0; i < w20e.Count; i++)
            {
                v_weights[i] = w20e[i][0];
            }

            double[] y_weights = new double[w40.Count];

            for (int i = 0; i < w40.Count; i++)
            {
                y_weights[i] = w40[i][0];
            }

            // model

            // declare variables etc.

            Console.WriteLine("\n> Start building the model");

            Cplex new_model = new Cplex();


            INumVar[][] x_variables = new INumVar[x_weights.Length][];
            INumVar[][] v_variables = new INumVar[v_weights.Length][];
            INumVar[][][] y_variables = new INumVar[y_weights.Length][][];
            INumVar[][] u_variables = new INumVar[hub_set.Count][];
            INumVar[] z_variables = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
            INumVar[] t_variables = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);


            for (int i = 0; i < x_weights.Length; i++)
            {
                x_variables[i] = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
            }

            for (int i = 0; i < v_weights.Length; i++)
            {
                v_variables[i] = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
            }

            for (int i = 0; i < y_weights.Length; i++)
            {
                y_variables[i] = new INumVar[car_amount][];

                for (int j = 0; j < car_amount; j++)
                {

                    y_variables[i][j] = new_model.NumVarArray(stack_amount, 0, int.MaxValue, NumVarType.Bool);
                }
            }

            for (int i = 0; i < hub_set.Count; i++)
            {
                u_variables[i] = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
            }

            // declare the objective function

            Console.WriteLine("[ 0/14]::Building objective function");




            // declare the objective function


            ILinearNumExpr y_total_cost = new_model.LinearNumExpr();


            for (int i = 0; i < y_weights.Length; i++)
            {
                for (int j = 0; j < car_amount; j++)
                {

                    y_total_cost.AddTerm(1, y_variables[i][j][0]);

                }
            }

            new_model.AddMaximize(y_total_cost);

            // declare constraints

            int x_amount = x_weights.Length, v_amount = v_weights.Length, y_amount = y_weights.Length;

            // constraint 2a

            Console.WriteLine("[ 1/14]::Building constraint1a");

            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < car_amount; i++)
                {
                    ILinearNumExpr constraint2a = new_model.LinearNumExpr();

                    for (int j = 0; j < x_amount; j++)
                    {
                        if (w20l[j][1] == hub_set[h])
                        {
                            constraint2a.AddTerm(1, x_variables[j][i]);
                        }
                    }

                    constraint2a.AddTerm(-2, z_variables[i]);

                    new_model.AddLe(constraint2a, 0);

                    constraint2a.Clear();

                }
            }
            // constraint 2b

            Console.WriteLine("[ 1/14]::Building constraint1b");

            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < car_amount; i++)
                {
                    ILinearNumExpr constraint2b = new_model.LinearNumExpr();

                    for (int j = 0; j < x_amount; j++)
                    {
                        if (w20l[j][1] == hub_set[h])
                        {
                            constraint2b.AddTerm(1, x_variables[j][i]);
                        }
                    }

                    constraint2b.AddTerm(-2, u_variables[h][i]);

                    new_model.AddLe(constraint2b, 0);

                    constraint2b.Clear();

                }
            }

            // constraint 2c

            Console.WriteLine("[ 1/14]::Building constraint1c");

            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < car_amount; i++)
                {
                    ILinearNumExpr constraint2c = new_model.LinearNumExpr();

                    for (int j = 0; j < x_amount; j++)
                    {
                        if (w20l[j][1] == hub_set[h])
                        {
                            constraint2c.AddTerm(1, x_variables[j][i]);
                        }
                    }

                    constraint2c.AddTerm(-2, u_variables[h][i]);
                    constraint2c.AddTerm(-2, z_variables[i]);

                    new_model.AddGe(constraint2c, -2);

                    constraint2c.Clear();

                }
            }

            // constraint 3

            Console.WriteLine("[ 2/14]::Building constraint2");

            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < x_amount; i++)
                {
                    if (w20l[i][1] == hub_set[h])
                    {
                        ILinearNumExpr constraint3 = new_model.LinearNumExpr();

                        for (int j = 0; j < car_amount; j++)
                        {
                            constraint3.AddTerm(1, x_variables[i][j]);
                        }

                        new_model.AddLe(constraint3, 1);

                        constraint3.Clear();
                    }
                }
            }

            // constraint 4a 

            Console.WriteLine("[ 3/14]::Building constraint3a");

            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < car_amount; i++)
                {
                    ILinearNumExpr constraint4a = new_model.LinearNumExpr();

                    for (int j = 0; j < v_amount; j++)
                    {
                        if (w20e[j][1] == hub_set[h])
                        {
                            constraint4a.AddTerm(1, v_variables[j][i]);
                        }
                    }

                    constraint4a.AddTerm(-2, t_variables[i]);

                    new_model.AddLe(constraint4a, 0);

                    constraint4a.Clear();

                }
            }

            // constraint 4b

            Console.WriteLine("[ 3/14]::Building constraint3b");

            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < car_amount; i++)
                {
                    ILinearNumExpr constraint4b = new_model.LinearNumExpr();

                    for (int j = 0; j < v_amount; j++)
                    {
                        if (w20e[j][1] == hub_set[h])
                        {
                            constraint4b.AddTerm(1, v_variables[j][i]);
                        }
                    }

                    constraint4b.AddTerm(-2, u_variables[h][i]);

                    new_model.AddLe(constraint4b, 0);

                    constraint4b.Clear();

                }
            }

            // constraint 4c

            Console.WriteLine("[ 3/14]::Building constraint3c");

            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < car_amount; i++)
                {
                    ILinearNumExpr constraint4c = new_model.LinearNumExpr();

                    for (int j = 0; j < v_amount; j++)
                    {
                        if (w20e[j][1] == hub_set[h])
                        {
                            constraint4c.AddTerm(1, v_variables[j][i]);
                        }
                    }

                    constraint4c.AddTerm(-2, u_variables[h][i]);
                    constraint4c.AddTerm(-2, t_variables[i]);

                    new_model.AddGe(constraint4c, -2);

                    constraint4c.Clear();

                }
            }

            // constraint 5 

            Console.WriteLine("[ 4/14]::Building constraint4");

            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < v_amount; i++)
                {
                    if (w20e[i][1] == hub_set[h])
                    {
                        ILinearNumExpr constraint5 = new_model.LinearNumExpr();

                        for (int j = 0; j < car_amount; j++)
                        {
                            constraint5.AddTerm(1, v_variables[i][j]);
                        }


                        new_model.AddLe(constraint5, 1);

                        constraint5.Clear();
                    }
                }
            }
            // constraint6 

            Console.WriteLine("[ 5/14]::Building constraint5");

            for (int i = 0; i < car_amount; i++)
            {
                ILinearNumExpr constraint6 = new_model.LinearNumExpr();

                constraint6.AddTerm(1, z_variables[i]);
                constraint6.AddTerm(1, t_variables[i]);

                new_model.AddLe(constraint6, 1);

                constraint6.Clear();
            }


            // constraint 7

            Console.WriteLine("[ 6/14]::Building constraint6");

            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < y_amount; i++)
                {
                    if (w40[i][1] == hub_set[h])
                    {
                        ILinearNumExpr constraint7 = new_model.LinearNumExpr();

                        for (int j = 0; j < car_amount; j++)
                        {
                            for (int k = 0; k < stack_amount; k++)
                            {
                                constraint7.AddTerm(1, y_variables[i][j][k]);
                            }
                        }

                        new_model.AddLe(constraint7, 1);

                        constraint7.Clear();
                    }
                }
            }

            // constraint 8

            Console.WriteLine("[ 7/14]::Building constraint7");

            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < car_amount; i++)
                {
                    ILinearNumExpr constraint8 = new_model.LinearNumExpr();

                    for (int j = 0; j < y_amount; j++)
                    {
                        if (w40[j][1] == hub_set[h])
                        {
                            constraint8.AddTerm(1, y_variables[j][i][0]);
                        }
                    }

                    constraint8.AddTerm(-1, u_variables[h][i]);

                    new_model.AddLe(constraint8, 0);
                }
            }
            // constraint 9a

            Console.WriteLine("[ 8/14]::Building constraint8a");

            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < car_amount; i++)
                {
                    ILinearNumExpr constraint9a = new_model.LinearNumExpr();

                    for (int j = 0; j < y_amount; j++)
                    {
                        if (w40[j][1] == hub_set[h])
                        {
                            constraint9a.AddTerm(1, y_variables[j][i][1]);
                        }
                    }

                    constraint9a.AddTerm(1, z_variables[i]);

                    new_model.AddLe(constraint9a, 1);
                }
            }

            // constraint 9b

            Console.WriteLine("[ 8/14]::Building constraint8b");

            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < car_amount; i++)
                {
                    ILinearNumExpr constraint9b = new_model.LinearNumExpr();

                    for (int j = 0; j < y_amount; j++)
                    {

                        if (w40[j][1] == hub_set[h])
                        {
                            constraint9b.AddTerm(1, y_variables[j][i][1]);
                        }
                    }

                    constraint9b.AddTerm(-1, u_variables[h][i]);

                    new_model.AddLe(constraint9b, 0);
                }
            }

            // constraint 10a

            Console.WriteLine("[ 9/14]::Building constraint9");

            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < car_amount; i++)
                {
                    ILinearNumExpr constraint10a = new_model.LinearNumExpr();

                    for (int j = 0; j < y_amount; j++)
                    {
                        if (w40[j][1] == hub_set[h])
                        {
                            constraint10a.AddTerm(1, y_variables[j][i][1]);
                        }
                    }

                    constraint10a.AddTerm(1, t_variables[i]);

                    new_model.AddLe(constraint10a, 1);
                }
            }

            // constraint 10b

            Console.WriteLine("[10/14]::Building constraint10");

            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < car_amount; i++)
                {
                    ILinearNumExpr constraint10b = new_model.LinearNumExpr();

                    for (int j = 0; j < y_amount; j++)
                    {
                        if (w40[j][1] == hub_set[h])
                        {
                            constraint10b.AddTerm(1, y_variables[j][i][1]);
                        }
                    }

                    constraint10b.AddTerm(-1, u_variables[h][i]);

                    new_model.AddLe(constraint10b, 0);
                }
            }


            // constraint 11

            Console.WriteLine("[11/14]::Building constraint11");

            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < car_amount; i++)
                {
                    ILinearNumExpr constraint11 = new_model.LinearNumExpr();

                    for (int j = 0; j < y_amount; j++)
                    {
                        if (w40[j][1] == hub_set[h])
                        {
                            for (int k = 0; k < stack_amount; k++)
                            {
                                constraint11.AddTerm(y_weights[j], y_variables[j][i][k]);
                            }
                        }
                    }

                    for (int j = 0; j < x_amount; j++)
                    {
                        if (w20l[j][1] == hub_set[h])
                        {
                            constraint11.AddTerm(x_weights[j], x_variables[j][i]);
                        }
                    }

                    for (int j = 0; j < v_amount; j++)
                    {
                        if (w20e[j][1] == hub_set[h])
                        {
                            constraint11.AddTerm(v_weights[j], v_variables[j][i]);
                        }
                    }

                    new_model.AddLe(constraint11, Read_data.weight_limit[i]);

                    constraint11.Clear();
                }
            }

            // constraint 12

            Console.WriteLine("[12/14]::Building constraint12");

            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < car_amount; i++)
                {
                    ILinearNumExpr constraint12 = new_model.LinearNumExpr();

                    for (int j = 0; j < y_amount; j++)
                    {
                        if (w40[j][1] == hub_set[h])
                        {
                            constraint12.AddTerm(-1 * y_weights[j], y_variables[j][i][0]);
                            constraint12.AddTerm(Read_data.weight_tolerence_factor[i] * y_weights[j], y_variables[j][i][1]);
                        }
                    }

                    for (int j = 0; j < x_amount; j++)
                    {
                        if (w20l[j][1] == hub_set[h])
                        {
                            constraint12.AddTerm(Read_data.weight_tolerence_factor[i] * x_weights[j], x_variables[j][i]);
                        }
                    }

                    for (int j = 0; j < v_amount; j++)
                    {
                        if (w20e[j][1] == hub_set[h])
                        {
                            constraint12.AddTerm(Read_data.weight_tolerence_factor[i] * v_weights[j], v_variables[j][i]);
                        }
                    }

                    new_model.AddGe(constraint12, 0);

                    constraint12.Clear();
                }
            }


            //constraint 13

            Console.WriteLine("[13/14]::Building constraint13");

            for (int i = 0; i < car_amount; i++)
            {
                ILinearNumExpr constraint13 = new_model.LinearNumExpr();

                for (int h = 0; h < hub_amount; h++)
                {
                    constraint13.AddTerm(1, u_variables[h][i]);
                }

                new_model.AddEq(constraint13, 1);

                constraint13.Clear();
            }

            //constraint 14 NEW CONSTRAINT 19

            Console.WriteLine("[14/14]::Building constraint14");

            for (int i = 0; i < car_amount; i++)
            {
                ILinearNumExpr constraint14 = new_model.LinearNumExpr();

                for (int j = 0; j < x_amount; j++)
                {

                    constraint14.AddTerm(0.5, x_variables[j][i]);
                }

                for (int j = 0; j < v_amount; j++)
                {

                    constraint14.AddTerm(0.5, v_variables[j][i]);

                }

                for (int j = 0; j < y_amount; j++)
                {

                    constraint14.AddTerm(-1, y_variables[j][i][0]);
                    constraint14.AddTerm(1, y_variables[j][i][1]);

                }
                new_model.AddEq(constraint14, 0);

                constraint14.Clear();

            }


            // model-solving

            Console.WriteLine("\n> Start solving the model");

            System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();

            sw.Reset(); sw.Start();


            new_model.Solve();
            sw.Stop();

            double time1 = sw.Elapsed.TotalMilliseconds;

            // get the result variable values

            double[][] x_result = new double[x_amount][];

            for (int i = 0; i < x_amount; i++)
            {
                x_result[i] = new_model.GetValues(x_variables[i]);
            }

            double[][] v_result = new double[v_amount][];

            for (int i = 0; i < v_amount; i++)
            {
                v_result[i] = new_model.GetValues(v_variables[i]);
            }

            double[][][] y_result = new double[y_amount][][];

            for (int i = 0; i < y_amount; i++)
            {
                y_result[i] = new double[car_amount][];

                for (int j = 0; j < car_amount; j++)
                {
                    y_result[i][j] = new_model.GetValues(y_variables[i][j]);
                }
            }

            double[][] u_result = new double[hub_amount][];

            for (int i = 0; i < hub_amount; i++)
            {
                u_result[i] = new_model.GetValues(u_variables[i]);
            }

            // transform result into readable form


            double[][][] loading = new double[car_amount][][];

            for (int i = 0; i < car_amount; i++)
            {
                loading[i] = new double[6][];

                for (int j = 0; j < 6; j++)
                {
                    loading[i][j] = new double[2];
                }
            }

            List<List<double>> container_destination_list_x = new List<List<double>>();
            List<List<double>> container_destination_list_v = new List<List<double>>();
            List<List<double>> container_destination_list_y = new List<List<double>>();
            List<double> car_info_list = new List<double>();
            for (int i = 0; i < Read_data.car_info.Count; i++)
            {
                car_info_list.Add(i);
            }

            List<List<double>> car_info = Read_data.car_info;

            // record x-result
            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < x_amount; i++)
                {
                    for (int j = 0; j < car_amount; j++)
                    {
                        if (x_result[i][j] == 1 && u_result[h][j] == 1 && Function.find_destination(u_result, j) != -1)
                        {
                            if (loading[j][0][0] == 0)
                            {
                                loading[j][0][0] = i + 1;
                                loading[j][0][1] = x_weights[i];

                                container_destination_list_x.Add(new List<double> { i + 1, hub_set[Function.find_destination(u_result, j)] });

                            }


                            else
                            {
                                loading[j][1][0] = i + 1;
                                loading[j][1][1] = x_weights[i];
                                container_destination_list_x.Add(new List<double> { i + 1, hub_set[Function.find_destination(u_result, j)] }); ;
                            }
                        }
                    }
                }
            }



            // record v-result
            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < v_amount; i++)
                {
                    for (int j = 0; j < car_amount; j++)
                    {
                        if (v_result[i][j] == 1 && u_result[h][j] == 1 && Function.find_destination(u_result, j) != -1)
                        {
                            if (loading[j][2][0] == 0)
                            {
                                loading[j][2][0] = i + 1;
                                loading[j][2][1] = v_weights[i];
                                container_destination_list_v.Add(new List<double> { i + 1, hub_set[Function.find_destination(u_result, j)] });
                            }
                            else
                            {
                                loading[j][3][0] = i + 1;
                                loading[j][3][1] = v_weights[i];
                                container_destination_list_v.Add(new List<double> { i + 1, hub_set[Function.find_destination(u_result, j)] });
                            }
                        }
                    }
                }
            }

            // record y-result
            for (int h = 0; h < hub_amount; h++)
            {
                for (int i = 0; i < y_amount; i++)
                {
                    for (int j = 0; j < car_amount; j++)
                    {
                        if (y_result[i][j][1] == 1 && u_result[h][j] == 1 && Function.find_destination(u_result, j) != -1)
                        {
                            if (loading[j][4][0] == 0)
                            {
                                loading[j][4][0] = i + 1;
                                loading[j][4][1] = y_weights[i];
                                container_destination_list_y.Add(new List<double> { i + 1, hub_set[Function.find_destination(u_result, j)] });

                            }
                            else
                            {
                                Console.WriteLine("there's an error! #40's upper");
                                Console.Read();
                            }
                        }
                        if (y_result[i][j][0] == 1 && u_result[h][j] == 1)
                        {
                            if (loading[j][5][0] == 0)
                            {
                                loading[j][5][0] = i + 1;
                                loading[j][5][1] = y_weights[i];
                                container_destination_list_y.Add(new List<double> { i + 1, hub_set[Function.find_destination(u_result, j)] });
                            }
                            else
                            {
                                Console.WriteLine("there's an error! #40's lower");
                                Console.Read();
                            }
                        }
                    }
                }
            }

            List<double> w20l_container_list = new List<double>();
            List<double> w20e_container_list = new List<double>();
            List<double> w40_container_list = new List<double>();

            w20l_container_list = Function.container_amount(w20l);
            w20e_container_list = Function.container_amount(w20e);
            w40_container_list = Function.container_amount(w40);

            // output the result

            StreamWriter csv_output = new StreamWriter("first stage.csv");
            csv_output.WriteLine("First stage");
            csv_output.WriteLine(",20L,20L,20E,20E,40lower,40upper,,destination,weight utility,space utility");

            Console.WriteLine("\n+---result---+");
            List<double> one = new List<double>();


            for (int i = 0; i < car_amount; i++)
            {
                string output = "", temp = "", rank = "";
                double total_weight = 0;
                double space_utility = 0;

                for (int j = 0; j < 6; j++)
                {
                    if (loading[i][j][0] != 0)
                    {

                        switch (j)
                        {
                            case 0:
                            case 1:
                                temp = "x";
                                space_utility++;
                                w20l_container_list.Remove(loading[i][j][0]);

                                for (int k = 0; k <= w20l.Count - 1; k++)
                                {
                                    for (int x = 0; x < container_destination_list_x.Count; x++)
                                    {
                                        if (loading[i][j][0] == container_destination_list_x[x][0])
                                        {
                                            if (loading[i][j][1] == w20l[k][0] && container_destination_list_x[x][1] == w20l[k][1])
                                            {
                                                w20l.RemoveAt(k);
                                                k = w20l.Count;
                                                break;
                                            }
                                        }
                                    }
                                }

                                break;
                            case 2:
                            case 3:
                                temp = "v";
                                space_utility++;
                                w20e_container_list.Remove(loading[i][j][0]);

                                for (int k = 0; k <= w20e.Count - 1; k++)
                                {
                                    for (int x = 0; x < container_destination_list_v.Count; x++)
                                    {
                                        if (loading[i][j][0] == container_destination_list_v[x][0])
                                        {
                                            if (loading[i][j][1] == w20e[k][0] && container_destination_list_v[x][1] == w20e[k][1])
                                            {
                                                w20e.RemoveAt(k);
                                                k = w20e.Count;
                                                break;
                                            }
                                        }
                                    }
                                }
                                //}
                                break;
                            case 4:
                            case 5:
                                temp = "y";
                                space_utility += 2;
                                w40_container_list.Remove(loading[i][j][0]);

                                for (int k = 0; k <= w40.Count - 1; k++)
                                {
                                    for (int x = 0; x < container_destination_list_y.Count; x++)
                                    {
                                        if (loading[i][j][0] == container_destination_list_y[x][0])
                                        {
                                            if (loading[i][j][1] == w40[k][0] && container_destination_list_y[x][1] == w40[k][1])
                                            {
                                                w40.RemoveAt(k);
                                                k = w40.Count;
                                                break;
                                            }
                                        }
                                    }
                                }
                                break;
                        }

                        output += temp + loading[i][j][0].ToString();
                        total_weight += loading[i][j][1];

                    }
                    else
                    {
                        output += "-";
                    }
                    if (j < 5)
                    {
                        output += ", ";
                    }
                }

                switch (i)
                {
                    case 0:
                        rank = "st";
                        if (total_weight > 0)
                        {
                            for (int z = 0; z < car_info.Count; z++)
                            {
                                if (car_info_list[z] == i)
                                {
                                    car_info.RemoveAt(z);
                                    car_info_list.RemoveAt(z);
                                    break;

                                }
                            }

                        }
                        break;

                    case 1:
                        rank = "nd";
                        if (total_weight > 0)
                        {
                            for (int z = 0; z < car_info.Count; z++)
                            {
                                if (car_info_list[z] == i)
                                {
                                    car_info.RemoveAt(z);
                                    car_info_list.RemoveAt(z);
                                    break;

                                }
                            }
                        }

                        break;

                    case 2:
                        rank = "rd";
                        if (total_weight > 0)
                        {
                            for (int z = 0; z < car_info.Count; z++)
                            {
                                if (car_info_list[z] == i)
                                {
                                    car_info.RemoveAt(z);
                                    car_info_list.RemoveAt(z);
                                    break;

                                }
                            }
                        }

                        break;

                    default:
                        rank = "th";
                        if (total_weight > 0)
                        {
                            for (int z = 0; z < car_info.Count; z++)
                            {
                                if (car_info_list[z] == i)
                                {
                                    car_info.RemoveAt(z);
                                    car_info_list.RemoveAt(z);
                                    break;

                                }
                            }
                        }

                        break;
                }

                string destination;

                if (Function.find_destination(u_result, i) != -1)
                {
                    destination = hub_set[Function.find_destination(u_result, i)].ToString();
                }
                else
                {
                    destination = "none";
                }

                Console.WriteLine("| " + (i + 1).ToString() + rank + ": {" + output + "}:\n|\t> weight utility:(" + total_weight.ToString() + "/" +
                    Read_data.weight_limit[i].ToString() + "), space utility:(" + space_utility.ToString() + "/4), destination: hub[" +
                    destination + "]");

                csv_output.WriteLine((i + 1).ToString() + "," + output + ",," + destination + "," + total_weight.ToString() + "/" +
                    Read_data.weight_limit[i].ToString() + "," + (space_utility / 4).ToString());

            }


            double u1 = new_model.GetObjValue() / car_amount;
            Console.WriteLine("+------------+\n");
            Console.WriteLine("Finish solving, used time: " + (sw.Elapsed.TotalMilliseconds).ToString("0.####") +
                " milliseconds,objective value=" + new_model.GetObjValue().ToString("0.####") + "\n" + "utility: " + u1.ToString() + "\n");

            double ob1 = new_model.GetObjValue();

            csv_output.WriteLine("\ncontainers that are not on rail cars:");



            csv_output.WriteLine("(1) 20l:" + w20l_container_list.Count + " containers left");
            for (int i = 0; i <= w20l_container_list.Count - 1; i++)
            {
                csv_output.Write("x" + w20l_container_list[i] + ",");
            }

            csv_output.WriteLine();

            csv_output.WriteLine("(2) 20e:" + w20e_container_list.Count + " containers left");
            for (int i = 0; i <= w20e_container_list.Count - 1; i++)
            {
                csv_output.Write("v" + w20e_container_list[i] + ",");
            }

            csv_output.WriteLine();

            csv_output.WriteLine("(3) 40:" + w40_container_list.Count + " containers left");
            for (int i = 0; i <= w40_container_list.Count - 1; i++)
            {
                csv_output.Write("y" + w40_container_list[i] + ",");
            }

            csv_output.Write("\ntime used(s)," + (sw.Elapsed.TotalMilliseconds / 1000) + ",,obj-value," + new_model.GetObjValue() + "\n");

            csv_output.WriteLine("-\nSecond stage");

            int car_amount1 = (int)new_model.GetObjValue();
            car_amount = car_amount - (int)new_model.GetObjValue();

            new_model.End();

            #endregion
            #region second_stage

            double time2 = 0;
            if (car_amount > 0)
            {
                stack_amount = 1;
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

                x_amount = x_weights.Length;
                v_amount = v_weights.Length;
                y_amount = y_weights.Length;


                //////////
                new_model = new Cplex();

                x_variables = new INumVar[x_weights.Length][];
                v_variables = new INumVar[v_weights.Length][];
                y_variables = new INumVar[y_weights.Length][][];
                u_variables = new INumVar[hub_set.Count][];
                z_variables = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                t_variables = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);


                for (int i = 0; i < x_weights.Length; i++)
                {
                    x_variables[i] = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                }

                for (int i = 0; i < v_weights.Length; i++)
                {
                    v_variables[i] = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);

                }

                for (int i = 0; i < y_weights.Length; i++)
                {

                    y_variables[i] = new INumVar[car_amount][];

                    for (int j = 0; j < car_amount; j++)
                    {
                        y_variables[i][j] = new_model.NumVarArray(stack_amount, 0, int.MaxValue, NumVarType.Bool);
                    }

                }

                for (int i = 0; i < hub_set.Count; i++)
                {
                    u_variables[i] = new_model.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                }

                // declare the objective function

                Console.WriteLine("[ 0/14]::Building objective function");

                ILinearNumExpr x_total_cost = new_model.LinearNumExpr();
                ILinearNumExpr v_total_cost = new_model.LinearNumExpr();
                y_total_cost = new_model.LinearNumExpr();

                double multiplier = 1 / (4 * (double)car_amount);

                for (int i = 0; i < x_weights.Length; i++)
                {
                    for (int j = 0; j < car_amount; j++)
                    {
                        x_total_cost.AddTerm(multiplier, x_variables[i][j]);
                    }
                }

                for (int i = 0; i < v_weights.Length; i++)
                {
                    for (int j = 0; j < car_amount; j++)
                    {
                        v_total_cost.AddTerm(multiplier, v_variables[i][j]);
                    }
                }

                for (int i = 0; i < y_weights.Length; i++)
                {
                    for (int j = 0; j < car_amount; j++)
                    {

                        y_total_cost.AddTerm(2 * multiplier, y_variables[i][j][0]);

                    }
                }

                new_model.AddMaximize(new_model.Sum(new_model.Sum(x_total_cost, v_total_cost), y_total_cost));


                // constraint 2a 

                Console.WriteLine("[ 1/12]::Building constraint1a");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint2a = new_model.LinearNumExpr();

                        for (int j = 0; j < x_amount; j++)
                        {
                            if (w20l[j][1] == hub_set[h])
                            {
                                constraint2a.AddTerm(1, x_variables[j][i]);
                            }
                        }

                        constraint2a.AddTerm(-2, z_variables[i]);

                        new_model.AddLe(constraint2a, 0);

                        constraint2a.Clear();

                    }
                }
                // constraint 2b

                Console.WriteLine("[ 1/12]::Building constraint1b");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint2b = new_model.LinearNumExpr();

                        for (int j = 0; j < x_amount; j++)
                        {
                            if (w20l[j][1] == hub_set[h])
                            {
                                constraint2b.AddTerm(1, x_variables[j][i]);
                            }
                        }

                        constraint2b.AddTerm(-2, u_variables[h][i]);

                        new_model.AddLe(constraint2b, 0);

                        constraint2b.Clear();

                    }
                }

                // constraint 2c 

                Console.WriteLine("[ 1/12]::Building constraint1c");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint2c = new_model.LinearNumExpr();

                        for (int j = 0; j < x_amount; j++)
                        {
                            if (w20l[j][1] == hub_set[h])
                            {
                                constraint2c.AddTerm(1, x_variables[j][i]);
                            }
                        }

                        constraint2c.AddTerm(-2, u_variables[h][i]);
                        constraint2c.AddTerm(-2, z_variables[i]);

                        new_model.AddGe(constraint2c, -2);

                        constraint2c.Clear();

                    }
                }

                // constraint 3 

                Console.WriteLine("[ 2/12]::Building constraint2");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < x_amount; i++)
                    {
                        if (w20l[i][1] == hub_set[h])
                        {
                            ILinearNumExpr constraint3 = new_model.LinearNumExpr();

                            for (int j = 0; j < car_amount; j++)
                            {
                                constraint3.AddTerm(1, x_variables[i][j]);
                            }

                            new_model.AddLe(constraint3, 1);

                            constraint3.Clear();
                        }
                    }
                }

                // constraint 4a 

                Console.WriteLine("[ 3/12]::Building constraint3a");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint4a = new_model.LinearNumExpr();

                        for (int j = 0; j < v_amount; j++)
                        {
                            if (w20e[j][1] == hub_set[h])
                            {
                                constraint4a.AddTerm(1, v_variables[j][i]);
                            }
                        }

                        constraint4a.AddTerm(-2, t_variables[i]);

                        new_model.AddLe(constraint4a, 0);

                        constraint4a.Clear();

                    }
                }

                // constraint 4b

                Console.WriteLine("[ 3/12]::Building constraint3b");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint4b = new_model.LinearNumExpr();

                        for (int j = 0; j < v_amount; j++)
                        {
                            if (w20e[j][1] == hub_set[h])
                            {
                                constraint4b.AddTerm(1, v_variables[j][i]);
                            }
                        }

                        constraint4b.AddTerm(-2, u_variables[h][i]);

                        new_model.AddLe(constraint4b, 0);

                        constraint4b.Clear();

                    }
                }

                // constraint 4c

                Console.WriteLine("[ 3/12]::Building constraint3c");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint4c = new_model.LinearNumExpr();

                        for (int j = 0; j < v_amount; j++)
                        {
                            if (w20e[j][1] == hub_set[h])
                            {
                                constraint4c.AddTerm(1, v_variables[j][i]);
                            }
                        }

                        constraint4c.AddTerm(-2, u_variables[h][i]);
                        constraint4c.AddTerm(-2, t_variables[i]);

                        new_model.AddGe(constraint4c, -2);

                        constraint4c.Clear();

                    }
                }

                // constraint 5 

                Console.WriteLine("[ 4/12]::Building constraint4");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < v_amount; i++)
                    {
                        if (w20e[i][1] == hub_set[h])
                        {
                            ILinearNumExpr constraint5 = new_model.LinearNumExpr();

                            for (int j = 0; j < car_amount; j++)
                            {
                                constraint5.AddTerm(1, v_variables[i][j]);
                            }


                            new_model.AddLe(constraint5, 1);

                            constraint5.Clear();
                        }
                    }
                }
                // constraint 6 

                Console.WriteLine("[ 5/12]::Building constraint5");

                for (int i = 0; i < car_amount; i++)
                {
                    ILinearNumExpr constraint6 = new_model.LinearNumExpr();

                    constraint6.AddTerm(1, z_variables[i]);
                    constraint6.AddTerm(1, t_variables[i]);

                    new_model.AddLe(constraint6, 1);

                    constraint6.Clear();
                }


                // constraint 7

                Console.WriteLine("[ 6/12]::Building constraint6");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < y_amount; i++)
                    {
                        if (w40[i][1] == hub_set[h])
                        {
                            ILinearNumExpr constraint7 = new_model.LinearNumExpr();

                            for (int j = 0; j < car_amount; j++)
                            {
                                constraint7.AddTerm(1, y_variables[i][j][0]);

                            }

                            new_model.AddLe(constraint7, 1);

                            constraint7.Clear();
                        }
                    }
                }

                // constraint 8

                /*Console.WriteLine("[ 9/14]::Building constraint8");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint8 = new_model.LinearNumExpr();

                        for (int j = 0; j < y_amount; j++)
                        {
                            if (w40[j][1] == hub_set[h])
                            {
                                constraint8.AddTerm(1, y_variables[j][i][0]);
                            }
                        }

                        constraint8.AddTerm(-1, u_variables[h][i]);

                        new_model.AddLe(constraint8, 0);
                    }
                }*/
                // constraint 9a 

                Console.WriteLine("[ 7/12]::Building constraint7");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint9a = new_model.LinearNumExpr();

                        for (int j = 0; j < y_amount; j++)
                        {
                            if (w40[j][1] == hub_set[h])
                            {
                                constraint9a.AddTerm(1, y_variables[j][i][0]);
                            }
                        }

                        constraint9a.AddTerm(1, z_variables[i]);

                        new_model.AddLe(constraint9a, 1);
                    }
                }

                // constraint 9b

                Console.WriteLine("[ 8/12]::Building constraint8");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint9b = new_model.LinearNumExpr();

                        for (int j = 0; j < y_amount; j++)
                        {

                            if (w40[j][1] == hub_set[h])
                            {
                                constraint9b.AddTerm(1, y_variables[j][i][0]);
                            }
                        }

                        constraint9b.AddTerm(-1, u_variables[h][i]);

                        new_model.AddLe(constraint9b, 0);
                    }
                }

                // constraint 10a

                Console.WriteLine("[ 9/12]::Building constraint9");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint10a = new_model.LinearNumExpr();

                        for (int j = 0; j < y_amount; j++)
                        {
                            if (w40[j][1] == hub_set[h])
                            {
                                constraint10a.AddTerm(1, y_variables[j][i][0]);
                            }
                        }

                        constraint10a.AddTerm(1, t_variables[i]);

                        new_model.AddLe(constraint10a, 1);
                    }
                }

                // constraint 10b

                Console.WriteLine("[10/12]::Building constraint10");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint10b = new_model.LinearNumExpr();

                        for (int j = 0; j < y_amount; j++)
                        {
                            if (w40[j][1] == hub_set[h])
                            {
                                constraint10b.AddTerm(1, y_variables[j][i][0]);
                            }
                        }

                        constraint10b.AddTerm(-1, u_variables[h][i]);

                        new_model.AddLe(constraint10b, 0);
                    }
                }


                // constraint 11

                Console.WriteLine("[11/12]::Building constraint11");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint11 = new_model.LinearNumExpr();

                        for (int j = 0; j < y_amount; j++)
                        {
                            if (w40[j][1] == hub_set[h])
                            {

                                constraint11.AddTerm(y_weights[j], y_variables[j][i][0]);

                            }
                        }

                        for (int j = 0; j < x_amount; j++)
                        {
                            if (w20l[j][1] == hub_set[h])
                            {
                                constraint11.AddTerm(x_weights[j], x_variables[j][i]);
                            }
                        }

                        for (int j = 0; j < v_amount; j++)
                        {
                            if (w20e[j][1] == hub_set[h])
                            {
                                constraint11.AddTerm(v_weights[j], v_variables[j][i]);
                            }
                        }

                        new_model.AddLe(constraint11, Read_data.weight_limit[i]);

                        constraint11.Clear();
                    }
                }

                //constraint 13

                Console.WriteLine("[12/12]::Building constraint12");

                for (int i = 0; i < car_amount; i++)
                {
                    ILinearNumExpr constraint13 = new_model.LinearNumExpr();

                    for (int h = 0; h < hub_amount; h++)
                    {
                        constraint13.AddTerm(1, u_variables[h][i]);
                    }

                    new_model.AddEq(constraint13, 1);

                    constraint13.Clear();
                }

                Console.WriteLine("\n> Start solving the model again!!!");

                sw = new System.Diagnostics.Stopwatch();

                sw.Reset(); sw.Start();

                new_model.Solve();

                sw.Stop();

                // get the result variable values

                time2 = sw.Elapsed.TotalMilliseconds;
                double ob2 = new_model.GetObjValue();

                Console.WriteLine("\nFinish solving, used time: " + (sw.Elapsed.TotalMilliseconds).ToString("0.####") +
                    " milliseconds,objective value=" + new_model.GetObjValue().ToString("0.####"));

                // get the result variable values

                x_result = new double[x_amount][];

                for (int i = 0; i < x_amount; i++)
                {
                    x_result[i] = new_model.GetValues(x_variables[i]);
                }

                v_result = new double[v_amount][];

                for (int i = 0; i < v_amount; i++)
                {
                    v_result[i] = new_model.GetValues(v_variables[i]);
                }

                y_result = new double[y_amount][][];

                for (int i = 0; i < y_amount; i++)
                {
                    y_result[i] = new double[car_amount][];

                    for (int j = 0; j < car_amount; j++)
                    {
                        y_result[i][j] = new_model.GetValues(y_variables[i][j]);
                    }
                }

                double[][] u_result_second = new double[hub_amount][];

                for (int i = 0; i < hub_amount; i++)
                {
                    u_result_second[i] = new_model.GetValues(u_variables[i]);
                }


                // record x-result
                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < x_amount; i++)
                    {
                        for (int j = 0; j < car_amount; j++)
                        {
                            if (x_result[i][j] == 1 && u_result_second[h][j] == 1 && Function.find_destination(u_result_second, j) != -1)
                            {
                                if (loading[(int)car_info_list[j]][0][0] == 0)
                                {
                                    loading[(int)car_info_list[j]][0][0] = w20l_container_list[i];
                                    loading[(int)car_info_list[j]][0][1] = x_weights[i];

                                    container_destination_list_x.Add(new List<double> { i + 1, hub_set[Function.find_destination(u_result_second, j)] });

                                }


                                else
                                {
                                    loading[(int)car_info_list[j]][1][0] = w20l_container_list[i];
                                    loading[(int)car_info_list[j]][1][1] = x_weights[i];
                                    container_destination_list_x.Add(new List<double> { i + 1, hub_set[Function.find_destination(u_result_second, j)] }); ;
                                }
                            }
                        }
                    }
                }



                // record v-result
                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < v_amount; i++)
                    {
                        for (int j = 0; j < car_amount; j++)
                        {
                            if (v_result[i][j] == 1 && u_result_second[h][j] == 1 && Function.find_destination(u_result_second, j) != -1)
                            {
                                if (loading[(int)car_info_list[j]][2][0] == 0)
                                {
                                    loading[(int)car_info_list[j]][2][0] = w20e_container_list[i];
                                    loading[(int)car_info_list[j]][2][1] = v_weights[i];
                                    container_destination_list_v.Add(new List<double> { i + 1, hub_set[Function.find_destination(u_result_second, j)] });
                                }
                                else
                                {
                                    loading[(int)car_info_list[j]][3][0] = w20e_container_list[i];
                                    loading[(int)car_info_list[j]][3][1] = v_weights[i];
                                    container_destination_list_v.Add(new List<double> { i + 1, hub_set[Function.find_destination(u_result_second, j)] });
                                }
                            }
                        }
                    }
                }

                // record y-result

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < y_amount; i++)
                    {
                        for (int j = 0; j < car_amount; j++)
                        {
                            if (y_result[i][j][0] == 1 && u_result_second[h][j] == 1 && Function.find_destination(u_result_second, j) != -1)
                            {
                                if (loading[(int)car_info_list[j]][5][0] == 0)
                                {
                                    loading[(int)car_info_list[j]][4][0] = w40_container_list[i];
                                    loading[(int)car_info_list[j]][4][1] = y_weights[i];
                                    container_destination_list_y.Add(new List<double> { i + 1, hub_set[Function.find_destination(u_result_second, j)] });

                                }
                                else
                                {
                                    Console.WriteLine("there's an error! #40's upper");
                                    Console.Read();
                                }
                            }
                        }
                    }
                }


                Console.WriteLine("\n+---result---+");

                List<double> w20l_container_list_overall = w20l_container_list;
                List<double> w20e_container_list_overall = w20e_container_list;
                List<double> w40_container_list_overall = w40_container_list;

                w20l_container_list = Function.container_amount(w20l);
                w20e_container_list = Function.container_amount(w20e);
                w40_container_list = Function.container_amount(w40);

                for (int i = 0; i < loading.Length; i++)
                {
                    string output = "", temp = "", rank = "";
                    double total_weight = 0;
                    double space_utility = 0;

                    for (int j = 0; j < 6; j++)
                    {
                        if (loading[i][j][0] != 0)
                        {

                            switch (j)
                            {
                                case 0:
                                case 1:
                                    temp = "x";
                                    space_utility++;
                                    w20l_container_list_overall.Remove(loading[i][j][0]);


                                    for (int k = 0; k <= w20l.Count - 1; k++)
                                    {
                                        for (int x = 0; x < container_destination_list_x.Count; x++)
                                        {
                                            if (loading[i][j][0] == container_destination_list_x[x][0])
                                            {
                                                if (loading[i][j][1] == w20l[k][0] && container_destination_list_x[x][1] == w20l[k][1])
                                                {
                                                    w20l.RemoveAt(k);
                                                    k = w20l.Count;
                                                    break;
                                                }
                                            }
                                        }
                                    }

                                    break;
                                case 2:
                                case 3:
                                    temp = "v";
                                    space_utility++;

                                    w20e_container_list_overall.Remove(loading[i][j][0]);


                                    for (int k = 0; k <= w20e.Count - 1; k++)
                                    {
                                        for (int x = 0; x < container_destination_list_v.Count; x++)
                                        {
                                            if (loading[i][j][0] == container_destination_list_v[x][0])
                                            {
                                                if (loading[i][j][1] == w20e[k][0] && container_destination_list_v[x][1] == w20e[k][1])
                                                {
                                                    w20e.RemoveAt(k);
                                                    k = w20e.Count;
                                                    break;
                                                }
                                            }
                                        }
                                    }

                                    break;
                                case 4:
                                case 5:
                                    temp = "y";
                                    space_utility += 2;

                                    w40_container_list_overall.Remove(loading[i][j][0]);

                                    for (int k = 0; k <= w40.Count - 1; k++)
                                    {
                                        for (int x = 0; x < container_destination_list_y.Count; x++)
                                        {
                                            if (loading[i][j][0] == container_destination_list_y[x][0])
                                            {
                                                if (loading[i][j][1] == w40[k][0] && container_destination_list_y[x][1] == w40[k][1])
                                                {
                                                    w40.RemoveAt(k);
                                                    k = w40.Count;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                    break;
                            }

                            output += temp + loading[i][j][0].ToString();
                            total_weight += loading[i][j][1];

                        }
                        else
                        {
                            output += "-";
                        }
                        if (j < 5)
                        {
                            output += ", ";
                        }
                    }

                    switch (i)
                    {
                        case 0:
                            rank = "st";
                            if (total_weight > 0)
                            {
                                for (int z = 0; z < car_info.Count; z++)
                                {
                                    if (car_info_list[z] == i)
                                    {
                                        car_info.RemoveAt(z);
                                        car_info_list.RemoveAt(z);
                                        z = car_info.Count;
                                        break;

                                    }
                                }

                            }

                            break;

                        case 1:
                            rank = "nd";
                            if (total_weight > 0)
                            {
                                for (int z = 0; z < car_info.Count; z++)
                                {
                                    if (car_info_list[z] == i)
                                    {
                                        car_info.RemoveAt(z);
                                        car_info_list.RemoveAt(z);
                                        z = car_info.Count;
                                        break;

                                    }
                                }
                            }

                            break;

                        case 2:
                            rank = "rd";
                            if (total_weight > 0)
                            {
                                for (int z = 0; z < car_info.Count; z++)
                                {
                                    if (car_info_list[z] == i)
                                    {
                                        car_info.RemoveAt(z);
                                        car_info_list.RemoveAt(z);
                                        z = car_info.Count;
                                        break;

                                    }
                                }
                            }

                            break;

                        default:
                            rank = "th";
                            if (total_weight > 0)
                            {
                                for (int z = 0; z < car_info.Count; z++)
                                {
                                    if (car_info_list[z] == i)
                                    {
                                        car_info.RemoveAt(z);
                                        car_info_list.RemoveAt(z);
                                        z = car_info.Count;
                                        break;

                                    }
                                }
                            }

                            break;
                    }


                    Console.WriteLine("| " + (i + 1).ToString() + rank + ": {" + output + "}:\n|\t> weight utility:(" + total_weight.ToString() + "/" +
                        Read_data.weight_limit[i].ToString() + "), space utility:(" + space_utility.ToString() + "/4)");


                    csv_output.WriteLine((i + 1).ToString() + "," + output + ",," + "," + total_weight.ToString() + "/" +
                        Read_data.weight_limit[i].ToString() + "," + (space_utility / 4).ToString());

                }

                csv_output.WriteLine("\n(1) 20l:" + w20l_container_list_overall.Count + " containers left");
                List<List<double>> w20l_rest = new List<List<double>>();
                List<List<double>> w20e_rest = new List<List<double>>();
                List<List<double>> w40_rest = new List<List<double>>();

                for (int i = 0; i <= w20l_container_list_overall.Count - 1; i++)
                {
                    w20l_rest.Add(w20l_second[(int)w20l_container_list_overall[i] - 1]);
                    csv_output.Write("x" + w20l_container_list_overall[i] + ",");
                }

                csv_output.WriteLine();

                csv_output.WriteLine("(2) 20e:" + w20e_container_list_overall.Count + " containers left");
                for (int i = 0; i <= w20e_container_list_overall.Count - 1; i++)
                {
                    w20e_rest.Add(w20e_second[(int)w20e_container_list_overall[i] - 1]);
                    csv_output.Write("v" + w20e_container_list_overall[i] + ",");
                }

                csv_output.WriteLine();

                csv_output.WriteLine("(3) 40:" + w40_container_list_overall.Count + " containers left");
                for (int i = 0; i <= w40_container_list_overall.Count - 1; i++)
                {
                    w40_rest.Add(w40_second[(int)w40_container_list_overall[i] - 1]);
                    csv_output.Write("y" + w40_container_list_overall[i] + ",");
                }

                total_time_two_stage = time1 + time2;
                Console.WriteLine("+------------+\nFinish solving, used time: " + (sw.Elapsed.TotalMilliseconds).ToString("0.####") +
                " milliseconds,objective value=" + new_model.GetObjValue().ToString("0.####"));
                Console.WriteLine("\n+---Result---+\n| Total used-time:" + (total_time_two_stage / 1000).ToString("#.####") + " seconds");
                utility_first_stage = (car_amount1 + car_amount * ob2) / car_amount_all;
                Console.WriteLine("| Total utility:" + utility_first_stage + "\n+------------+");

                csv_output.WriteLine("\ntime-used," + time2 / 1000 + ",,obj-value," + new_model.GetObjValue().ToString(".##"));
                csv_output.WriteLine("-\nResult\ntime used," + total_time_two_stage / 1000 + ",,total utility," + utility_first_stage);

                csv_output.WriteLine("-\nThird stage");
                new_model.End();

                car_amount = car_info.Count;
                car_for_three_stage = car_amount;

                check_enter_three_stage = Function.check_stage2(car_amount, w20l_rest, w20e_rest, w40_rest, hub_set);

                if (check_enter_three_stage) Three_stage.model();
                
                new_model.End();
                #endregion

            }
        }
    }
}
