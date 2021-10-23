using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.Optimization;
using MathNet.Numerics.LinearAlgebra;

namespace RSUtils
{

    /// <summary>
    /// Implements a Limited Memory BFGS Optimizer with line search. that can be run using coroutines
    /// </summary>
    public class LBFGS
    {
        int max_m;                  // the number of previous optimization iterations to approximate the hessian by
        double grad_threshold;      // the lower bound for the gradient for termination
        public bool terminated;     // if the optimizer has terminated
        //int max_iter;
        //int iter;

        private List<Vector<double>> s;
        private List<Vector<double>> y;
        private List<double> alpha;
        private List<double> rho;



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

        /// <summary>
        /// finds a vector that minimizes the objective function. yield returns each time the objective function is evaluated to allow for deferred evaluation
        /// </summary>
        /// <param name="objective">the objective function to be minimized</param>
        /// <param name="initialGuess">the vector to start the optimization with</param>
        /// <returns></returns>
        public IEnumerable<Vector<double>> FindMinimum(IObjectiveFunction objective, Vector<double> initialGuess)
        {

            // check if already terminated
            while (terminated)
            {
                yield return initialGuess;
            }

            objective.EvaluateAt(initialGuess);

            yield return initialGuess;

            // check gradient termination criterium
            if(objective.Gradient.L1Norm() < grad_threshold * initialGuess.Count)
            {
                Debug.Log("LBFGS optimizer terminated: gradient threshold criterium");
                terminated = true;
            }

            // initialize optimization variables
            int m = s.Count;
            var q = objective.Gradient;
            alpha.Clear();
            rho.Clear();

            // approximate hessian
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

            // do line search
            foreach (var tmp in LineSearch(objective, initialGuess, -z))
            {
                x_k1 = tmp;

                yield return initialGuess;
            }
            //var x_k1 = LineSearch(objective, initialGuess, -z);
            //objective.EvaluateAt(x_k1);
            //yield return initialGuess;

            var g_k1 = objective.Gradient;

            // cleanup internal state for next iteration
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

        /// <summary>
        /// does a non-backtracking line search and returns on success. if not successful after 5 iterations, returns initial guess
        /// </summary>
        /// <param name="objective">the objective function</param>
        /// <param name="position">the starting position of the line search</param>
        /// <param name="direction">the search direction of the line search</param>
        /// <returns></returns>
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

        /// <summary>
        /// resets the optimization and starts fresh
        /// </summary>
        public void Reset()
        {
            terminated = false;
        }
    }
}
