using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using MathNet.Numerics.LinearAlgebra;
//using Microsoft.DirectX;

namespace Trajectory
{
    // minlm_d_vb example: http://www.alglib.net/translator/man/manual.csharp.html#example_minlm_d_v
    class Optimizer
    {
        public NeedleKinematics kinematics;
        public Matrix3D T_taget;
        private Matrix3D T_FK;
        
        public Optimizer()
        {
            kinematics = new NeedleKinematics();
            Matrix<double> A;

        }
        public double[] minimize_error()
        {
            double[] x = new double[] { 0, 0, 0, 0 };
            double epsg = 0.0000000000001;
            double epsf = 0;
            double epsx = 0;
            int maxits = 0;
            alglib.minlmstate state;
            alglib.minlmreport rep;

            alglib.minlmcreatev(16, x, 0.0000000001, out state);
            alglib.minlmsetcond(state, epsg, epsf, epsx, maxits);
            alglib.minlmoptimize(state, function_fvec, null, null);
            alglib.minlmresults(state, out x, out rep);
            
            double[] output=new double[4];   
            output[0] = x[0];
            output[1] = x[1];
            output[2] = x[2];
            output[3] = x[3];

            // updating joints
            kinematics.leftUpperBevel = x[0];
            kinematics.leftLowerBevel = x[1];
            kinematics.leftElbow = x[2];
            kinematics.twist = x[3];
            // calculating the forward kinematics for the given joints value
            T_FK = kinematics.transformation_matrix(55);

            return output;

            

        }
        private void function_fvec(double[] x, double[] fi, object obj)
        {
            //
            // this callback calculates
            // f0(x0,x1) = 100*(x0+3)^4,
            // f1(x0,x1) = (x1-3)^4
            //

            // updating joints
            kinematics.leftUpperBevel = x[0];
            kinematics.leftLowerBevel = x[1];
            kinematics.leftElbow = x[2];
            kinematics.twist = x[3];
            // calculating the forward kinematics for the given joints value
            T_FK = kinematics.transformation_matrix(55);
            // calculating errors
            fi[0] = Math.Pow(T_taget.M11 - T_FK.M11, 2);
            fi[1] = Math.Pow(T_taget.M12 - T_FK.M12, 2);
            fi[2] = Math.Pow(T_taget.M13 - T_FK.M13, 2);
            fi[3] = Math.Pow(T_taget.M14 - T_FK.M14, 2);
            fi[4] = Math.Pow(T_taget.M21 - T_FK.M21, 2);
            fi[5] = Math.Pow(T_taget.M22 - T_FK.M22, 2);
            fi[6] = Math.Pow(T_taget.M23 - T_FK.M23, 2);
            fi[7] = Math.Pow(T_taget.M24 - T_FK.M24, 2);
            fi[8] = Math.Pow(T_taget.M31 - T_FK.M31, 2);
            fi[9] = Math.Pow(T_taget.M32 - T_FK.M32, 2);
            fi[10] = Math.Pow(T_taget.M33 - T_FK.M33, 2);
            fi[11] = Math.Pow(T_taget.M34 - T_FK.M34, 2);
        }
    }
}
