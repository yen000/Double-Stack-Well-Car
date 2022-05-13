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
    class Original_model
    {
        public static List<List<double>> w20_original = Read_data.w20l;
        public static List<List<double>> w20e_original = Read_data.w20e;
        public static List<List<double>> w40_original = Read_data.w40;
        public static void model()
        {
            Cplex new_model = new Cplex();
            List<double> hub_set = Function.get_hub_sets(w20_original, w20e_original, w40_original);
            Console.WriteLine("j " + hub_set.Count);

            #region Declare variable
            // declare variables etc.
            INumVar[][] x_variables = new INumVar[Read_data.x_weights.Length][];
            INumVar[][] v_variables = new INumVar[Read_data.v_weights.Length][];
            INumVar[][][] y_variables = new INumVar[Read_data.y_weights.Length][][];
            INumVar[][] u_variables = new INumVar[hub_set.Count][];
            INumVar[] z_variables = new_model.NumVarArray(Read_data.car_amount, 0, int.MaxValue, NumVarType.Bool);
            INumVar[] t_variables = new_model.NumVarArray(Read_data.car_amount, 0, int.MaxValue, NumVarType.Bool);


            for (int i = 0; i < Read_data.x_weights.Length; i++)
            {
                x_variables[i] = new_model.NumVarArray(Read_data.car_amount, 0, int.MaxValue, NumVarType.Bool);
            }

            for (int i = 0; i < Read_data.v_weights.Length; i++)
            {
                v_variables[i] = new_model.NumVarArray(Read_data.car_amount, 0, int.MaxValue, NumVarType.Bool);
            }

            for (int i = 0; i < Read_data.y_weights.Length; i++)
            {

                y_variables[i] = new INumVar[Read_data.car_amount][];

                for (int j = 0; j < Read_data.car_amount; j++)
                {
                    y_variables[i][j] = new_model.NumVarArray(Read_data.stack_amount, 0, int.MaxValue, NumVarType.Bool);
                }

            }

            for (int i = 0; i < hub_set.Count; i++)
            {
                u_variables[i] = new_model.NumVarArray(Read_data.car_amount, 0, int.MaxValue, NumVarType.Bool);
            }

            #endregion

            #region Objective function
            // declare the objective function
            Console.WriteLine("[ 0/14]::Building objective function");

            ILinearNumExpr x_total_cost = new_model.LinearNumExpr();
            ILinearNumExpr v_total_cost = new_model.LinearNumExpr();
            ILinearNumExpr y_total_cost = new_model.LinearNumExpr();

            double multiplier = 1 / (4 * (double)Read_data.car_amount);

            for (int i = 0; i < Read_data.x_weights.Length; i++)
            {
                for (int j = 0; j < Read_data.car_amount; j++)
                {
                    x_total_cost.AddTerm(multiplier, x_variables[i][j]);
                }
            }

            for (int i = 0; i < Read_data.v_weights.Length; i++)
            {
                for (int j = 0; j < Read_data.car_amount; j++)
                {
                    v_total_cost.AddTerm(multiplier, v_variables[i][j]);
                }
            }

            for (int i = 0; i < Read_data.y_weights.Length; i++)
            {
                for (int j = 0; j < Read_data.car_amount; j++)
                {
                    for (int k = 0; k < Read_data.stack_amount; k++)
                    {
                        y_total_cost.AddTerm(2 * multiplier, y_variables[i][j][k]);
                    }
                }
            }

            new_model.AddMaximize(new_model.Sum(new_model.Sum(x_total_cost, v_total_cost), y_total_cost));
            #endregion

            #region Constraint
            // declare constraints

            int x_amount = Read_data.x_weights.Length, v_amount = Read_data.v_weights.Length, y_amount = Read_data.y_weights.Length;

            // constraint 2a 

            Console.WriteLine("[ 1/12]::Building constraint1");

            for (int h = 0; h < Read_data.hub_amount; h++)
            {
                for (int i = 0; i < Read_data.car_amount; i++)
                {
                    ILinearNumExpr constraint2a = new_model.LinearNumExpr();

                    for (int j = 0; j < x_amount; j++)
                    {
                        if (w20_original[j][1] == hub_set[h])
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

            Console.WriteLine("[ 1/12]::Building constraint1");

            for (int h = 0; h < Read_data.hub_amount; h++)
            {
                for (int i = 0; i < Read_data.car_amount; i++)
                {
                    ILinearNumExpr constraint2b = new_model.LinearNumExpr();

                    for (int j = 0; j < x_amount; j++)
                    {
                        if (w20_original[j][1] == hub_set[h])
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

            Console.WriteLine("[ 1/12]::Building constraint1");

            for (int h = 0; h < Read_data.hub_amount; h++)
            {
                for (int i = 0; i < Read_data.car_amount; i++)
                {
                    ILinearNumExpr constraint2c = new_model.LinearNumExpr();

                    for (int j = 0; j < x_amount; j++)
                    {
                        if (w20_original[j][1] == hub_set[h])
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

            // constraint3

            Console.WriteLine("[ 2/12]::Building constraint3");

            for (int h = 0; h < Read_data.hub_amount; h++)
            {
                for (int i = 0; i < x_amount; i++)
                {
                    if (w20_original[i][1] == hub_set[h])
                    {
                        ILinearNumExpr constraint3 = new_model.LinearNumExpr();

                        for (int j = 0; j < Read_data.car_amount; j++)
                        {
                            constraint3.AddTerm(1, x_variables[i][j]);
                        }

                        new_model.AddLe(constraint3, 1);

                        constraint3.Clear();
                    }
                }
            }

            // constraint 4a 

            Console.WriteLine("[ 1/12]::Building constraint1");

            for (int h = 0; h < Read_data.hub_amount; h++)
            {
                for (int i = 0; i < Read_data.car_amount; i++)
                {
                    ILinearNumExpr constraint4a = new_model.LinearNumExpr();

                    for (int j = 0; j < v_amount; j++)
                    {
                        if (w20e_original[j][1] == hub_set[h])
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

            Console.WriteLine("[ 1/12]::Building constraint1");

            for (int h = 0; h < Read_data.hub_amount; h++)
            {
                for (int i = 0; i < Read_data.car_amount; i++)
                {
                    ILinearNumExpr constraint4b = new_model.LinearNumExpr();

                    for (int j = 0; j < v_amount; j++)
                    {
                        if (w20e_original[j][1] == hub_set[h])
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

            Console.WriteLine("[ 1/12]::Building constraint1");

            for (int h = 0; h < Read_data.hub_amount; h++)
            {
                for (int i = 0; i < Read_data.car_amount; i++)
                {
                    ILinearNumExpr constraint4c = new_model.LinearNumExpr();

                    for (int j = 0; j < v_amount; j++)
                    {
                        if (w20e_original[j][1] == hub_set[h])
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

            // constraint5

            Console.WriteLine("[ 4/12]::Building constraint5");

            for (int h = 0; h < Read_data.hub_amount; h++)
            {
                for (int i = 0; i < v_amount; i++)
                {
                    if (w20e_original[i][1] == hub_set[h])
                    {
                        ILinearNumExpr constraint5 = new_model.LinearNumExpr();

                        for (int j = 0; j < Read_data.car_amount; j++)
                        {
                            constraint5.AddTerm(1, v_variables[i][j]);
                        }


                        new_model.AddLe(constraint5, 1);

                        constraint5.Clear();
                    }
                }
            }
            // constraint6

            Console.WriteLine("[ 4/14]::Building constraint4-1");

            for (int i = 0; i < Read_data.car_amount; i++)
            {
                ILinearNumExpr constraint6 = new_model.LinearNumExpr();

                constraint6.AddTerm(1, z_variables[i]);
                constraint6.AddTerm(1, t_variables[i]);

                new_model.AddLe(constraint6, 1);

                constraint6.Clear();
            }


            // constraint 7

            Console.WriteLine("[ 8/14]::Building constraint7");

            for (int h = 0; h < Read_data.hub_amount; h++)
            {
                for (int i = 0; i < y_amount; i++)
                {
                    if (w40_original[i][1] == hub_set[h])
                    {
                        ILinearNumExpr constraint7 = new_model.LinearNumExpr();

                        for (int j = 0; j < Read_data.car_amount; j++)
                        {
                            for (int k = 0; k < Read_data.stack_amount; k++)
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

            Console.WriteLine("[ 9/14]::Building constraint8");

            for (int h = 0; h < Read_data.hub_amount; h++)
            {
                for (int i = 0; i < Read_data.car_amount; i++)
                {
                    ILinearNumExpr constraint8 = new_model.LinearNumExpr();

                    for (int j = 0; j < y_amount; j++)
                    {
                        if (w40_original[j][1] == hub_set[h])
                        {
                            constraint8.AddTerm(1, y_variables[j][i][0]);
                        }
                    }

                    constraint8.AddTerm(-1, u_variables[h][i]);

                    new_model.AddLe(constraint8, 0);
                }
            }
            // constraint 9a

            Console.WriteLine("[10/14]::Building constraint11");

            for (int h = 0; h < Read_data.hub_amount; h++)
            {
                for (int i = 0; i < Read_data.car_amount; i++)
                {
                    ILinearNumExpr constraint9a = new_model.LinearNumExpr();

                    for (int j = 0; j < y_amount; j++)
                    {
                        if (w40_original[j][1] == hub_set[h])
                        {
                            constraint9a.AddTerm(1, y_variables[j][i][1]);
                        }
                    }

                    constraint9a.AddTerm(1, z_variables[i]);

                    new_model.AddLe(constraint9a, 1);
                }
            }

            // constraint 9b

            Console.WriteLine("[10/14]::Building constraint11");

            for (int h = 0; h < Read_data.hub_amount; h++)
            {
                for (int i = 0; i < Read_data.car_amount; i++)
                {
                    ILinearNumExpr constraint9b = new_model.LinearNumExpr();

                    for (int j = 0; j < y_amount; j++)
                    {

                        if (w40_original[j][1] == hub_set[h])
                        {
                            constraint9b.AddTerm(1, y_variables[j][i][1]);
                        }
                    }

                    constraint9b.AddTerm(-1, u_variables[h][i]);

                    new_model.AddLe(constraint9b, 0);
                }
            }

            // constraint 10a

            Console.WriteLine("[10/14]::Building constraint11");

            for (int h = 0; h < Read_data.hub_amount; h++)
            {
                for (int i = 0; i < Read_data.car_amount; i++)
                {
                    ILinearNumExpr constraint10a = new_model.LinearNumExpr();

                    for (int j = 0; j < y_amount; j++)
                    {
                        if (w40_original[j][1] == hub_set[h])
                        {
                            constraint10a.AddTerm(1, y_variables[j][i][1]);
                        }
                    }

                    constraint10a.AddTerm(1, t_variables[i]);

                    new_model.AddLe(constraint10a, 1);
                }
            }

            // constraint 10b

            Console.WriteLine("[10/14]::Building constraint11");

            for (int h = 0; h < Read_data.hub_amount; h++)
            {
                for (int i = 0; i < Read_data.car_amount; i++)
                {
                    ILinearNumExpr constraint10b = new_model.LinearNumExpr();

                    for (int j = 0; j < y_amount; j++)
                    {
                        if (w40_original[j][1] == hub_set[h])
                        {
                            constraint10b.AddTerm(1, y_variables[j][i][1]);
                        }
                    }

                    constraint10b.AddTerm(-1, u_variables[h][i]);

                    new_model.AddLe(constraint10b, 0);
                }
            }


            // constraint 11

            Console.WriteLine("[12/14]::Building constraint11");

            for (int h = 0; h < Read_data.hub_amount; h++)
            {
                for (int i = 0; i < Read_data.car_amount; i++)
                {
                    ILinearNumExpr constraint11 = new_model.LinearNumExpr();

                    for (int j = 0; j < y_amount; j++)
                    {
                        if (w40_original[j][1] == hub_set[h])
                        {
                            for (int k = 0; k < Read_data.stack_amount; k++)
                            {
                                constraint11.AddTerm(Read_data.y_weights[j], y_variables[j][i][k]);
                            }
                        }
                    }

                    for (int j = 0; j < x_amount; j++)
                    {
                        if (w20_original[j][1] == hub_set[h])
                        {
                            constraint11.AddTerm(Read_data.x_weights[j], x_variables[j][i]);
                        }
                    }

                    for (int j = 0; j < v_amount; j++)
                    {
                        if (w20e_original[j][1] == hub_set[h])
                        {
                            constraint11.AddTerm(Read_data.v_weights[j], v_variables[j][i]);
                        }
                    }

                    new_model.AddLe(constraint11, Read_data.weight_limit[i]);

                    constraint11.Clear();
                }
            }

            // constraint 12 

            Console.WriteLine("[13/14]::Building constraint12");

            for (int h = 0; h < Read_data.hub_amount; h++)
            {
                for (int i = 0; i < Read_data.car_amount; i++)
                {
                    ILinearNumExpr constraint12 = new_model.LinearNumExpr();

                    for (int j = 0; j < y_amount; j++)
                    {
                        if (w40_original[j][1] == hub_set[h])
                        {
                            constraint12.AddTerm(-1 * Read_data.y_weights[j], y_variables[j][i][0]);
                            constraint12.AddTerm(Read_data.weight_tolerence_factor[i] * Read_data.y_weights[j], y_variables[j][i][1]);
                        }
                    }

                    for (int j = 0; j < x_amount; j++)
                    {
                        if (w20_original[j][1] == hub_set[h])
                        {
                            constraint12.AddTerm(Read_data.weight_tolerence_factor[i] * Read_data.x_weights[j], x_variables[j][i]);
                        }
                    }

                    for (int j = 0; j < v_amount; j++)
                    {
                        if (w20e_original[j][1] == hub_set[h])
                        {
                            constraint12.AddTerm(Read_data.weight_tolerence_factor[i] * Read_data.v_weights[j], v_variables[j][i]);
                        }
                    }

                    new_model.AddGe(constraint12, 0);

                    constraint12.Clear();
                }
            }


            //constraint 13

            Console.WriteLine("[14/14]::Building constraint13");

            for (int i = 0; i < Read_data.car_amount; i++)
            {
                ILinearNumExpr constraint13 = new_model.LinearNumExpr();

                for (int h = 0; h < Read_data.hub_amount; h++)
                {
                    constraint13.AddTerm(1, u_variables[h][i]);
                }

                new_model.AddEq(constraint13, 1);

                constraint13.Clear();
            }

            #endregion

            #region Model solving
            // model-solving
            Console.WriteLine("\n> Start solving the model");

            System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();

            sw.Reset(); sw.Start();

            new_model.Solve();

            sw.Stop();
            #endregion

            #region Result output
            // get the result variable values
            Console.WriteLine("\nFinish solving, used time: " + (sw.Elapsed.TotalMilliseconds).ToString("0.####") +
                " milliseconds,objective value=" + new_model.GetObjValue().ToString("0.####"));

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
                y_result[i] = new double[Read_data.car_amount][];

                for (int j = 0; j < Read_data.car_amount; j++)
                {
                    y_result[i][j] = new_model.GetValues(y_variables[i][j]);
                }
            }

            double[][] u_result = new double[Read_data.hub_amount][];

            for (int i = 0; i < Read_data.hub_amount; i++)
            {
                u_result[i] = new_model.GetValues(u_variables[i]);
            }

            // transform result into readable form

            double[][][] loading = new double[Read_data.car_amount][][];

            for (int i = 0; i < Read_data.car_amount; i++)
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
                for (int j = 0; j < Read_data.car_amount; j++)
                {
                    if (x_result[i][j] == 1)
                    {
                        if (loading[j][0][0] == 0)
                        {
                            loading[j][0][0] = i + 1;
                            loading[j][0][1] = Read_data.x_weights[i];
                        }
                        else
                        {
                            loading[j][1][0] = i + 1;
                            loading[j][1][1] = Read_data.x_weights[i];
                        }
                    }
                }
            }

            // record v-result

            for (int i = 0; i < v_amount; i++)
            {
                for (int j = 0; j < Read_data.car_amount; j++)
                {
                    if (v_result[i][j] == 1)
                    {
                        if (loading[j][2][0] == 0)
                        {
                            loading[j][2][0] = i + 1;
                            loading[j][2][1] = Read_data.v_weights[i];
                        }
                        else
                        {
                            loading[j][3][0] = i + 1;
                            loading[j][3][1] = Read_data.v_weights[i];
                        }
                    }
                }
            }

            // record y-result

            for (int i = 0; i < y_amount; i++)
            {
                for (int j = 0; j < Read_data.car_amount; j++)
                {
                    if (y_result[i][j][1] == 1)
                    {
                        loading[j][4][0] = i + 1;
                        loading[j][4][1] = Read_data.y_weights[i];
                    }
                    if (y_result[i][j][0] == 1)
                    {
                        loading[j][5][0] = i + 1;
                        loading[j][5][1] = Read_data.y_weights[i];
                    }
                }
            }

            // output the result

            string result_file_path = "Original_rsult.csv";
            StreamWriter csv_output = new StreamWriter(result_file_path);
            csv_output.WriteLine(",20L,20L,20E,20E,40lower,40upper,,destination,weight utility,space utility");

            Console.WriteLine("\n+---result---+");

            for (int i = 0; i < Read_data.car_amount; i++)
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
                    Read_data.weight_limit[i].ToString() + "), space utility:(" + space_utility.ToString() + "/4), destination: hub[" +
                    destination + "]");

                csv_output.WriteLine((i + 1).ToString() + "," + output + ",," + destination + "," + total_weight.ToString() + "/" +
                    Read_data.weight_limit[i].ToString() + "," + (space_utility / 4).ToString());

            }

            csv_output.Write("\ntime used(ms)," + (sw.Elapsed.TotalMilliseconds / 60000) + ",,obj-value," + new_model.GetObjValue());

            csv_output.Close();
            #endregion

            new_model.End();
        }
    }
}
