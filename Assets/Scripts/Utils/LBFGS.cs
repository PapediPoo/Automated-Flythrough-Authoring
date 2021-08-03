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
        double grad_threshold;
        public bool terminated;
        //int max_iter;
        //int iter;

        List<Vector<double>> s;
        List<Vector<double>> y;
        readonly List<double> alpha;
        readonly List<double> rho;



        public LBFGS(int m, double gradient_termination_threshold)
        {
            this.max_m = m;
            this.grad_threshold = gradient_termination_threshold;
            terminated = false;
            //this.max_iter = max_iter;
            //this.iter = 0;

            s = new List<Vector<double>>();
            y = new List<Vector<double>>();

            alpha = new List<double>();
            rho = new List<double>();
        }
        public IEnumerable<Vector<double>> FindMinimum(IObjectiveFunction objective, Vector<double> initialGuess)
        {
            while (terminated)
            {
                yield return initialGuess;
            }

            //if(iter >= max_iter)
            //{
            //    return initialGuess;
            //}
            objective.EvaluateAt(initialGuess);

            yield return initialGuess;

            if(objective.Gradient.L1Norm() < grad_threshold * initialGuess.Count)
            {
                Debug.Log("LBFGS optimizer terminated: gradient threshold criterium");
                terminated = true;
            }

            int m = s.Count;
            var q = objective.Gradient;
            var alpha = new List<double>();
            var rho = new List<double>();

            for (int i = 0; i < m; i++)
            {
                var rho_i = 1d / y[i].DotProduct(s[i]);
                var alpha_i = rho_i * s[i].DotProduct(q);
                q -= (alpha_i * y[i]);

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
            Vector<double> x_k1 = initialGuess;
            yield return initialGuess;

            foreach (var tmp in LineSearch(objective, initialGuess, -z))
            {
                x_k1 = tmp;

                yield return initialGuess;
            }
            //var x_k1 = LineSearch(objective, initialGuess, -z);
            //objective.EvaluateAt(x_k1);
            //yield return initialGuess;

            var g_k1 = objective.Gradient;


            s.Add(x_k1 - x_k);
            y.Add(g_k1 - g_k);


            if (s.Count > max_m)
            {
                s.RemoveAt(0);
                y.RemoveAt(0);
            }

            //return initialGuess;
            yield return x_k1;
        }

        private IEnumerable<Vector<double>> LineSearch(IObjectiveFunction objective, Vector<double> position, Vector<double> direction)
        {
            // objective = objective.CreateNew();
            double factor = 1;

            //if (!objective.Point.Equals(position))
            //{
            //    objective.EvaluateAt(position);
            //}
            double init_val = objective.Value;
            var init_point = objective.Point;
            double next_val;
            int MAX_ITERATIONS = 5;
            int current_iteration = 0;

            do
            {
                yield return init_point;
                objective.EvaluateAt(position + (factor * direction));
                next_val = objective.Value;
                factor *= 0.5d;
                current_iteration++;
            } while (next_val >= init_val && current_iteration <= MAX_ITERATIONS);
            // objective.EvaluateAt(position + (factor * direction));

            yield return objective.Point;
        }

        public void Reset()
        {
            terminated = false;
        }
    }
}
