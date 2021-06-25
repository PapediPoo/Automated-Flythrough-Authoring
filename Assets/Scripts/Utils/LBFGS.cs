using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.Optimization;
using MathNet.Numerics.LinearAlgebra;

namespace RSUtils
{
    public class LBFGS
    {
        int max_m;
        List<Vector<double>> s;
        List<Vector<double>> y;

        List<double> alpha;
        List<double> rho;


        public LBFGS(int m)
        {
            this.max_m = m;
            s = new List<Vector<double>>();
            y = new List<Vector<double>>();

            alpha = new List<double>();
            rho = new List<double>();

        }
        public Vector<double> FindMinimum(IObjectiveFunction objective, Vector<double> initialGuess)
        {
            objective.EvaluateAt(initialGuess);


            int m = s.Count;
            var q = objective.Gradient;
            var alpha = new List<double>();
            var rho = new List<double>();

            for (int i = 0; i < m; i++)
            {
                var rho_i = 1d / y[i].DotProduct(s[i]);
                var alpha_i = rho_i * s[i].DotProduct(q);
                q = q - (alpha_i * y[i]);

                alpha.Add(alpha_i);
                rho.Add(rho_i);
            }

            double gamma_k;
            if (m > 0)
            {
                gamma_k = s[m - 1].DotProduct(y[m - 1]) / y[m - 1].DotProduct(y[m - 1]);
            }
            else
            {
                gamma_k = 1;
            }
            //var hessian_0 = gamma_k * Matrix<double>.Build.SparseIdentity(n, n);
            var z = gamma_k * q;

            for (int i = 0; i < m; i++)
            {
                var beta_i = rho[i] * y[i].DotProduct(z);
                z += s[i] * (alpha[i] - beta_i);
            }


            var x_k = initialGuess;
            var g_k = objective.Gradient;
            //var x_k1 = initialGuess - (z * 0.333);
            var x_k1 = LineSearch(objective, initialGuess, -z);
            objective.EvaluateAt(x_k1);
            var g_k1 = objective.Gradient;


            s.Add(x_k1 - x_k);
            y.Add(g_k1 - g_k);


            if (s.Count > max_m)
            {
                s.RemoveAt(0);
                y.RemoveAt(0);
            }

            // Debug.Log(s.Count);
            // Debug.Log(x_k1.At(0));

            //return initialGuess;
            return x_k1;
        }

        private Vector<double> LineSearch(IObjectiveFunction objective, Vector<double> position, Vector<double> direction)
        {
            objective = objective.CreateNew();
            double factor = 1;

            //if (!objective.Point.Equals(position))
            //{
            //    objective.EvaluateAt(position);
            //}
            double init_val = objective.Value;
            double next_val;

            do
            {
                objective.EvaluateAt(position + (factor * direction));
                next_val = objective.Value;
                factor *= 0.5d;
            } while (next_val >= init_val);
            // objective.EvaluateAt(position + (factor * direction));

            return objective.Point;
        }
    }
}
