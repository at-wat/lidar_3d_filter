#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class hokuyo3d_compensate_node
{
	public:
		hokuyo3d_compensate_node() :
			nh("~")
		{
			for(int i = 0; i < 8; i ++)
			{
				auto num = std::to_string(i);
				sensor s;
				if(!nh.hasParam("frame_id" + num)) break;
				nh.param("frame_id" + num, s.frame_id, std::string(""));
				nh.param("phase_shift" + num, s.phase_shift, 0.0);
				nh.param("amp_mul" + num, s.amp_mul, 1.0);
				nh.param("range_mul" + num, s.range_mul, 1.0);
				nh.param("range_offset" + num, s.range_offset, 0.0);
				nh.param("gain_amp" + num, s.gain_amp, 1.0);
				nh.param("gain_phase" + num, s.gain_phase, 1.0);
				std::string mode;
				nh.param("mode" + num, mode, std::string("auto"));
				if(mode.compare("auto") == 0)
					s.mode = comp_mode::AUTO;
				else if(mode.compare("manual") == 0)
					s.mode = comp_mode::MANUAL;
				else if(mode.compare("auto_phase") == 0)
					s.mode = comp_mode::AUTO_PHASE;
				else
				{
					ROS_ERROR("unknown hokuyo3d compensate mode (auto or manual)");
					ros::shutdown();
					return;
				}

				ROS_INFO("%s compensation: phase %0.3f, amp %0.2f", 
						s.frame_id.c_str(), s.phase_shift, s.amp_mul);
				sensors.push_back(s);
			}

			sub_pc = nh.subscribe("cloud_in", 1, &hokuyo3d_compensate_node::cb_cloud, this);
			pub_pc = nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 1, true);
			phase_cache.reset(new float[1024]);
			for(int i = 0; i < 1024; i ++) phase_cache[i] = FLT_MAX;
		}
	private:
		void cb_cloud(const sensor_msgs::PointCloud2::Ptr &msg)
		{
			double *phase_shift;
			double *amp_mul;
			int *cnt;
			comp_mode mode;
			float range_offset;
			float range_mul;
			float gain_amp;
			float gain_phase;
			bool found = false;
			for(auto &s: sensors)
			{
				if(msg->header.frame_id.compare(s.frame_id) == 0)
				{
					phase_shift = &s.phase_shift;
					amp_mul = &s.amp_mul;
					cnt = &s.cnt;
					range_offset = s.range_offset;
					range_mul = s.range_mul;
					mode = s.mode;
					gain_amp = s.gain_amp;
					gain_phase = s.gain_phase;
					found = true;
					break;
				}
			}
			if(!found)
			{
				pub_pc.publish(*msg);
				return;
			}

			auto pc = *msg;
			float *data = reinterpret_cast<float*>(&msg->data[0]);
			float *data2 = reinterpret_cast<float*>(&pc.data[0]);
			float phase_prev = 0;
			bool neg = false;
			float z_up = 0.0;
			int z_up_num = 0;
			float z_down = 0.0;
			int z_down_num = 0;
			float z_d = 0.0;
			int z_d_num = 0;
			for(unsigned int j = 0; j < msg->width; j ++)
			{
				float x = *(data++);
				float y = *(data++);
				float z = *(data++);
				//float i = *(data++);
				data ++;
				float r2 = powf(x, 2.0) + powf(y, 2.0);
				float r = r2 + powf(z, 2.0);
				r2 = sqrtf(r2);
				r = sqrtf(r);

				if(r == 0.0)
				{
					data2 += 4;
					continue;
				}
			//	float yaw = atan2f(y, x);
				int index = (((z / r) + 0.1) / 0.005);
				
				float rem_phase, phase;
				if(phase_cache[index] == FLT_MAX)
				{
					float pitch = asinf(z / r);
					phase = asinf((pitch - M_PI/12.0) / (M_PI*20.0/180.0));
					phase_cache[index] = phase;
				}
				else
				{
					phase = phase_cache[index];
				}
				if((phase - phase_prev < -0.01 || 
							(fabs(phase - phase_prev) < 0.01 && neg)) && 
						phase > -1.45)
				{
					neg = true;
					if(phase > 0.0) rem_phase = M_PI/2.0 - phase + M_PI/2.0;
					else rem_phase = -M_PI/2.0 - phase - M_PI/2.0;
				}
				else
				{
					neg = false;
					rem_phase = phase;
				}

				float pitch2 = sinf(rem_phase + (*phase_shift)) * (*amp_mul)
				   	* (M_PI*20.0/180.0) + M_PI/12.0;
				z = sinf(pitch2) * (r * range_mul + range_offset);

				if(r2 > 1.0)
				{
					if(cosf(rem_phase + *phase_shift) > 0)
					{
						z_up += z / r2;
						z_up_num ++;
					}
					else
					{
						z_down += z / r2;
						z_down_num ++;
					}
					if(fabs(z) < 0.25)
					{
						z_d += z / r2;
						z_d_num ++;
					}
				}

			//	printf("%f %f %f %f %f %f\n", yaw, pitch, z/r, rem_phase, phase, phase_prev);
				phase_prev = phase;

				data2 += 2;
				*(data2++) = z;
				data2 ++;
			}
			z_up /= z_up_num;
			z_down /= z_down_num;
			z_d /= z_d_num;
			float err = z_up - z_down;
			if(mode != comp_mode::MANUAL)
			{
				if(std::isfinite(err) && std::isfinite(z_d))
				{
					*phase_shift -= err * 0.1 * gain_phase;
					if(mode == comp_mode::AUTO)
						*amp_mul += (z_d - 0.05) * 0.025 * gain_amp;
				}
				
				if((*cnt)++ % 128 == 0)
				{
					ROS_INFO("%s compensation: phase %0.3f, amp %0.2f", 
							msg->header.frame_id.c_str(), (float)*phase_shift, (float)*amp_mul);
				}
			}
			//printf("%0.3f  %0.3f   %0.3f %0.3f\n", z_up - z_down, *phase_shift, z_d, *amp_mul);
			pub_pc.publish(pc);
		}
		ros::NodeHandle nh;
		ros::Publisher pub_pc;
		ros::Subscriber sub_pc;

		enum comp_mode{
			AUTO,
			AUTO_PHASE,
			MANUAL
		};
		class sensor{
		public:
			std::string frame_id;
			double phase_shift;
			double amp_mul;
			double range_mul;
			double range_offset;
			double gain_amp;
			double gain_phase;
			int cnt;
			comp_mode mode;
		};
		std::vector<sensor> sensors;
		std::unique_ptr<float[]> phase_cache;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "hokuyo3d_compensate");
	hokuyo3d_compensate_node node;

	ros::spin();

	return 1;
}

