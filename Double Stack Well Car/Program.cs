 using System;

namespace Double_Stack_Well_Car
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("<Program start>");

            string file_name = "dataset";

            Read_data.model(file_name);
            Original_model.model();
            Two_stage.model();

            Console.WriteLine("<Program end>");
            Console.Read();
        }
    }
}
