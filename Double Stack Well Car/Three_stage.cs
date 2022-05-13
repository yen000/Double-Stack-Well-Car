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
    class Three_stage
    {
        public static List<List<double>> w20l = new List<List<double>>();
        public static List<List<double>> w20e = new List<List<double>>();
        public static List<List<double>> w40 = new List<List<double>>();
        public static List<List<double>> w20l_second = new List<List<double>>();
        public static List<List<double>> w20e_second = new List<List<double>>();
        public static List<List<double>> w40_second = new List<List<double>>();
        public static void model()
        {
            w20l = Read_data.w20l;
            w20e = Read_data.w20e;
            w40 = Read_data.w40;
            w20l_second = Read_data.w20l;
            w20e_second = Read_data.w20e;
            w40_second = Read_data.w40;

            #region third_stage
            if (Two_stage.check_enter_three_stage == true)
            {
                int stack_amount = 2;
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

                List<double> hub_set = Function.get_hub_sets(w20l, w20e, w40);

                int hub_amount = hub_set.Count;
                int car_amount = Read_data.car_amount;

                int x_amount = x_weights.Length;
                int v_amount = v_weights.Length;
                int y_amount = y_weights.Length;
                car_amount = Read_data.car_info_second.Count;

                double[]weight_limit = new double[car_amount];
                double[]weight_tolerence_factor = new double[car_amount];

                for (int i = 0; i < car_amount; i++)
                {
                    weight_limit[i] = Read_data.car_info_second[i][0];
                    weight_tolerence_factor[i] = Read_data.car_info_second[i][1];
                }
                //////////////////////////////////////////
                Cplex new_model2 = new Cplex();

                INumVar[][] x_variables = new INumVar[x_weights.Length][];
                INumVar[][] v_variables = new INumVar[v_weights.Length][];
                INumVar[][][] y_variables = new INumVar[y_weights.Length][][];
                INumVar[][] u_variables = new INumVar[hub_set.Count][];
                INumVar[] z_variables = new_model2.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                INumVar[] t_variables = new_model2.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);


                for (int i = 0; i < x_weights.Length; i++)
                {
                    x_variables[i] = new_model2.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                }

                for (int i = 0; i < v_weights.Length; i++)
                {
                    v_variables[i] = new_model2.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                }

                for (int i = 0; i < y_weights.Length; i++)
                {

                    y_variables[i] = new INumVar[car_amount][];

                    for (int j = 0; j < car_amount; j++)
                    {
                        y_variables[i][j] = new_model2.NumVarArray(stack_amount, 0, int.MaxValue, NumVarType.Bool);
                    }

                }

                for (int i = 0; i < hub_set.Count; i++)
                {
                    u_variables[i] = new_model2.NumVarArray(car_amount, 0, int.MaxValue, NumVarType.Bool);
                }
                double u2 =Function. utility_upper_bound(Two_stage.car_for_three_stage, Read_data.car_amount, Two_stage.utility_first_stage, Two_stage.w20l_rest, Two_stage.w20e_rest, Two_stage.w40_rest, hub_set);

                // declare the objective function

                Console.WriteLine("[ 0/14]::Building objective function");

                ILinearNumExpr x_total_cost = new_model2.LinearNumExpr();
                ILinearNumExpr v_total_cost = new_model2.LinearNumExpr();
                ILinearNumExpr y_total_cost = new_model2.LinearNumExpr();

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
                        for (int k = 0; k < stack_amount; k++)
                        {
                            y_total_cost.AddTerm(2 * multiplier, y_variables[i][j][k]);
                        }

                    }
                }

                new_model2.AddMaximize(new_model2.Sum(new_model2.Sum(x_total_cost, v_total_cost), y_total_cost));

                new_model2.AddLe((new_model2.Sum(new_model2.Sum(x_total_cost, v_total_cost), y_total_cost)), u2);
                new_model2.AddGe((new_model2.Sum(new_model2.Sum(x_total_cost, v_total_cost), y_total_cost)), Two_stage.utility_first_stage);

                Console.WriteLine("[ 1/12]::Building constraint1");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint2a = new_model2.LinearNumExpr();

                        for (int j = 0; j < x_amount; j++)
                        {
                            if (w20l_second[j][1] == hub_set[h])
                            {
                                constraint2a.AddTerm(1, x_variables[j][i]);
                            }
                        }

                        constraint2a.AddTerm(-2, z_variables[i]);

                        new_model2.AddLe(constraint2a, 0);

                        constraint2a.Clear();

                    }
                }
                // constraint 2b

                Console.WriteLine("[ 1/12]::Building constraint1");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint2b = new_model2.LinearNumExpr();

                        for (int j = 0; j < x_amount; j++)
                        {
                            if (w20l_second[j][1] == hub_set[h])
                            {
                                constraint2b.AddTerm(1, x_variables[j][i]);
                            }
                        }

                        constraint2b.AddTerm(-2, u_variables[h][i]);

                        new_model2.AddLe(constraint2b, 0);

                        constraint2b.Clear();

                    }
                }

                // constraint 2c

                Console.WriteLine("[ 1/12]::Building constraint1");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint2b = new_model2.LinearNumExpr();

                        for (int j = 0; j < x_amount; j++)
                        {
                            if (w20l_second[j][1] == hub_set[h])
                            {
                                constraint2b.AddTerm(1, x_variables[j][i]);
                            }
                        }

                        constraint2b.AddTerm(-2, u_variables[h][i]);
                        constraint2b.AddTerm(-2, z_variables[i]);

                        new_model2.AddGe(constraint2b, -2);

                        constraint2b.Clear();

                    }
                }

                // constraint 3

                Console.WriteLine("[ 2/12]::Building constraint3");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < x_amount; i++)
                    {
                        if (w20l_second[i][1] == hub_set[h])
                        {
                            ILinearNumExpr constraint3 = new_model2.LinearNumExpr();

                            for (int j = 0; j < car_amount; j++)
                            {
                                constraint3.AddTerm(1, x_variables[i][j]);
                            }

                            new_model2.AddLe(constraint3, 1);

                            constraint3.Clear();
                        }
                    }
                }

                // constraint 4a 

                Console.WriteLine("[ 1/12]::Building constraint1");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint4a = new_model2.LinearNumExpr();

                        for (int j = 0; j < v_amount; j++)
                        {
                            if (w20e_second[j][1] == hub_set[h])
                            {
                                constraint4a.AddTerm(1, v_variables[j][i]);
                            }
                        }

                        constraint4a.AddTerm(-2, t_variables[i]);

                        new_model2.AddLe(constraint4a, 0);

                        constraint4a.Clear();

                    }
                }

                // constraint 4b 

                Console.WriteLine("[ 1/12]::Building constraint1");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint4b = new_model2.LinearNumExpr();

                        for (int j = 0; j < v_amount; j++)
                        {
                            if (w20e_second[j][1] == hub_set[h])
                            {
                                constraint4b.AddTerm(1, v_variables[j][i]);
                            }
                        }

                        constraint4b.AddTerm(-2, u_variables[h][i]);

                        new_model2.AddLe(constraint4b, 0);

                        constraint4b.Clear();

                    }
                }

                // constraint 4c

                Console.WriteLine("[ 1/12]::Building constraint1");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint4c = new_model2.LinearNumExpr();

                        for (int j = 0; j < v_amount; j++)
                        {
                            if (w20e_second[j][1] == hub_set[h])
                            {
                                constraint4c.AddTerm(1, v_variables[j][i]);
                            }
                        }

                        constraint4c.AddTerm(-2, u_variables[h][i]);
                        constraint4c.AddTerm(-2, t_variables[i]);

                        new_model2.AddGe(constraint4c, -2);

                        constraint4c.Clear();

                    }
                }

                // constraint 5

                Console.WriteLine("[ 4/12]::Building constraint5");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < v_amount; i++)
                    {
                        if (w20e_second[i][1] == hub_set[h])
                        {
                            ILinearNumExpr constraint5 = new_model2.LinearNumExpr();

                            for (int j = 0; j < car_amount; j++)
                            {
                                constraint5.AddTerm(1, v_variables[i][j]);
                            }


                            new_model2.AddLe(constraint5, 1);

                            constraint5.Clear();
                        }
                    }
                }
                // constraint 6

                Console.WriteLine("[ 4/14]::Building constraint4-1");

                for (int i = 0; i < car_amount; i++)
                {
                    ILinearNumExpr constraint6 = new_model2.LinearNumExpr();

                    constraint6.AddTerm(1, z_variables[i]);
                    constraint6.AddTerm(1, t_variables[i]);

                    new_model2.AddLe(constraint6, 1);

                    constraint6.Clear();
                }


                // constraint 7

                Console.WriteLine("[ 8/14]::Building constraint7");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < y_amount; i++)
                    {
                        if (w40_second[i][1] == hub_set[h])
                        {
                            ILinearNumExpr constraint7 = new_model2.LinearNumExpr();

                            for (int j = 0; j < car_amount; j++)
                            {
                                for (int k = 0; k < stack_amount; k++)
                                {
                                    constraint7.AddTerm(1, y_variables[i][j][k]);
                                }
                            }

                            new_model2.AddLe(constraint7, 1);

                            constraint7.Clear();
                        }
                    }
                }

                // constraint 8 

                Console.WriteLine("[ 9/14]::Building constraint8");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint8 = new_model2.LinearNumExpr();

                        for (int j = 0; j < y_amount; j++)
                        {
                            if (w40_second[j][1] == hub_set[h])
                            {
                                constraint8.AddTerm(1, y_variables[j][i][0]);
                            }
                        }

                        constraint8.AddTerm(-1, u_variables[h][i]);

                        new_model2.AddLe(constraint8, 0);
                    }
                }
                // constraint 9a

                Console.WriteLine("[10/14]::Building constraint11");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint9a = new_model2.LinearNumExpr();

                        for (int j = 0; j < y_amount; j++)
                        {
                            if (w40_second[j][1] == hub_set[h])
                            {
                                constraint9a.AddTerm(1, y_variables[j][i][1]);
                            }
                        }

                        constraint9a.AddTerm(1, z_variables[i]);

                        new_model2.AddLe(constraint9a, 1);
                    }
                }

                // constraint 9b

                Console.WriteLine("[10/14]::Building constraint11");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint9b = new_model2.LinearNumExpr();

                        for (int j = 0; j < y_amount; j++)
                        {

                            if (w40_second[j][1] == hub_set[h])
                            {
                                constraint9b.AddTerm(1, y_variables[j][i][1]);
                            }
                        }

                        constraint9b.AddTerm(-1, u_variables[h][i]);

                        new_model2.AddLe(constraint9b, 0);
                    }
                }

                // constraint 10a

                Console.WriteLine("[10/14]::Building constraint11");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint10a = new_model2.LinearNumExpr();

                        for (int j = 0; j < y_amount; j++)
                        {
                            if (w40_second[j][1] == hub_set[h])
                            {
                                constraint10a.AddTerm(1, y_variables[j][i][1]);
                            }
                        }

                        constraint10a.AddTerm(1, t_variables[i]);

                        new_model2.AddLe(constraint10a, 1);
                    }
                }

                // constraint 10b

                Console.WriteLine("[10/14]::Building constraint11");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint10b = new_model2.LinearNumExpr();

                        for (int j = 0; j < y_amount; j++)
                        {
                            if (w40_second[j][1] == hub_set[h])
                            {
                                constraint10b.AddTerm(1, y_variables[j][i][1]);
                            }
                        }

                        constraint10b.AddTerm(-1, u_variables[h][i]);

                        new_model2.AddLe(constraint10b, 0);
                    }
                }


                // constraint 11

                Console.WriteLine("[12/14]::Building constraint11");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint11 = new_model2.LinearNumExpr();

                        for (int j = 0; j < y_amount; j++)
                        {
                            if (w40_second[j][1] == hub_set[h])
                            {
                                for (int k = 0; k < stack_amount; k++)
                                {
                                    constraint11.AddTerm(y_weights[j], y_variables[j][i][k]);
                                }
                            }
                        }

                        for (int j = 0; j < x_amount; j++)
                        {
                            if (w20l_second[j][1] == hub_set[h])
                            {
                                constraint11.AddTerm(x_weights[j], x_variables[j][i]);
                            }
                        }

                        for (int j = 0; j < v_amount; j++)
                        {
                            if (w20e_second[j][1] == hub_set[h])
                            {
                                constraint11.AddTerm(v_weights[j], v_variables[j][i]);
                            }
                        }

                        new_model2.AddLe(constraint11, weight_limit[i]);

                        constraint11.Clear();
                    }
                }

                // constraint 12 

                Console.WriteLine("[13/14]::Building constraint12");

                for (int h = 0; h < hub_amount; h++)
                {
                    for (int i = 0; i < car_amount; i++)
                    {
                        ILinearNumExpr constraint12 = new_model2.LinearNumExpr();

                        for (int j = 0; j < y_amount; j++)
                        {
                            if (w40_second[j][1] == hub_set[h])
                            {
                                constraint12.AddTerm(-1 * y_weights[j], y_variables[j][i][0]);
                                constraint12.AddTerm(weight_tolerence_factor[i] * y_weights[j], y_variables[j][i][1]);
                            }
                        }

                        for (int j = 0; j < x_amount; j++)
                        {
                            if (w20l_second[j][1] == hub_set[h])
                            {
                                constraint12.AddTerm(weight_tolerence_factor[i] * x_weights[j], x_variables[j][i]);
                            }
                        }

                        for (int j = 0; j < v_amount; j++)
                        {
                            if (w20e_second[j][1] == hub_set[h])
                            {
                                constraint12.AddTerm(weight_tolerence_factor[i] * v_weights[j], v_variables[j][i]);
                            }
                        }

                        new_model2.AddGe(constraint12, 0);

                        constraint12.Clear();
                    }
                }


                //constraint 13

                Console.WriteLine("[14/14]::Building constraint13");

                for (int i = 0; i < car_amount; i++)
                {
                    ILinearNumExpr constraint13 = new_model2.LinearNumExpr();

                    for (int h = 0; h < hub_amount; h++)
                    {
                        constraint13.AddTerm(1, u_variables[h][i]);
                    }

                    new_model2.AddEq(constraint13, 1);

                    constraint13.Clear();
                }


                // model-solving

                Console.WriteLine("\n> Start solving the model");

                System.Diagnostics.Stopwatch gg = new System.Diagnostics.Stopwatch();

                gg.Reset(); gg.Start();

                new_model2.Solve();

                gg.Stop();


                Console.WriteLine("\nFinish solving, used time: " + (gg.Elapsed.TotalMilliseconds).ToString("0.####") +
                    " milliseconds,objective value=" + new_model2.GetObjValue().ToString("0.####"));
                double time3 = gg.Elapsed.TotalMilliseconds;
                double total_time = Two_stage.total_time_two_stage + time3;
                double ob3 = new_model2.GetObjValue();

                // get the result variable values

                double[][] x_result = new double[x_amount][];

                for (int i = 0; i < x_amount; i++)
                {
                    x_result[i] = new_model2.GetValues(x_variables[i]);
                }

                double[][] v_result = new double[v_amount][];

                for (int i = 0; i < v_amount; i++)
                {
                    v_result[i] = new_model2.GetValues(v_variables[i]);
                }

                double[][][] y_result = new double[y_amount][][];

                for (int i = 0; i < y_amount; i++)
                {
                    y_result[i] = new double[car_amount][];

                    for (int j = 0; j < car_amount; j++)
                    {
                        y_result[i][j] = new_model2.GetValues(y_variables[i][j]);
                    }
                }

                double[][] u_result = new double[hub_amount][];

                for (int i = 0; i < hub_amount; i++)
                {
                    u_result[i] = new_model2.GetValues(u_variables[i]);
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


                // record x-result

                for (int i = 0; i < x_amount; i++)
                {
                    for (int j = 0; j < car_amount; j++)
                    {
                        if (x_result[i][j] == 1)
                        {
                            if (loading[j][0][0] == 0)
                            {
                                loading[j][0][0] = i + 1;
                                loading[j][0][1] = x_weights[i];
                            }
                            else
                            {
                                loading[j][1][0] = i + 1;
                                loading[j][1][1] = x_weights[i];
                            }
                        }
                    }
                }

                // record v-result

                for (int i = 0; i < v_amount; i++)
                {
                    for (int j = 0; j < car_amount; j++)
                    {
                        if (v_result[i][j] == 1)
                        {
                            if (loading[j][2][0] == 0)
                            {
                                loading[j][2][0] = i + 1;
                                loading[j][2][1] = v_weights[i];
                            }
                            else
                            {
                                loading[j][3][0] = i + 1;
                                loading[j][3][1] = v_weights[i];
                            }
                        }
                    }
                }

                // record y-result

                for (int i = 0; i < y_amount; i++)
                {
                    for (int j = 0; j < car_amount; j++)
                    {
                        if (y_result[i][j][1] == 1)
                        {
                            loading[j][4][0] = i + 1;
                            loading[j][4][1] = y_weights[i];
                        }
                        if (y_result[i][j][0] == 1)
                        {
                            loading[j][5][0] = i + 1;
                            loading[j][5][1] = y_weights[i];
                        }
                    }
                }

                // output the result
                StreamWriter csv_output = new StreamWriter("three_stage.csv");


                csv_output.WriteLine(",20L,20L,20E,20E,40lower,40upper,,destination,weight utility,space utility");

                Console.WriteLine("\n+---result---+");

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
                                    break;
                                case 2:
                                case 3:
                                    temp = "v";
                                    space_utility++;
                                    break;
                                case 4:
                                case 5:
                                    temp = "y";
                                    space_utility += 2;
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
                            break;
                        case 1:
                            rank = "nd";
                            break;
                        case 2:
                            rank = "rd";
                            break;
                        default:
                            rank = "th";
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
                        weight_limit[i].ToString() + "), space utility:(" + space_utility.ToString() + "/4), destination: hub[" +
                        destination + "]");

                    csv_output.WriteLine((i + 1).ToString() + "," + output + ",," + destination + "," + total_weight.ToString() + "/" +
                         weight_limit[i].ToString() + "," + (space_utility / 4).ToString());

                }

                csv_output.Write("\ntime used(ms)," + (gg.Elapsed.TotalMilliseconds / 1000) + ",,obj-value," + new_model2.GetObjValue());
                csv_output.Write("\n\ntotal time:," + total_time / 1000 + ",,final utitility:," + ob3);
                Console.WriteLine("\ntotal_time(sec)  " + total_time / 1000 + "\nutility:   " + ob3);
                new_model2.End();
            }
            #endregion
        }
    }
}
