using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using Microsoft.DirectX;
namespace Trajectory
{
    class Program
    {
        Needle needle;
        static void Main(string[] args)
        {
            // create and initialize the needle
            Needle needle = new Needle();
            needle.kinematics.joint.UpperBevel = 30;
            needle.kinematics.joint.LowerBevel = 30;
            needle.kinematics.joint.Elbow = 10;
            needle.kinematics.joint.twist = 0;
            needle.update_needle();

            for (int n = 0; n < 20; n++)
            {
                // optimizing
                Optimizer optimizer = new Optimizer();
                optimizer.T_taget = needle.moved_head; // must be updated

                // updating needle
                Joints optimized = optimizer.minimize_error();
                needle.kinematics.joint.UpperBevel = optimized.UpperBevel;
                needle.kinematics.joint.LowerBevel = optimized.LowerBevel;
                needle.kinematics.joint.Elbow = optimized.Elbow;
                needle.kinematics.joint.twist = optimized.twist;
                needle.update_needle();
            }
     
        }
    }
}
