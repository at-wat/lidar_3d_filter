#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <fstream>
#include <iostream>

class remove_myself_node
{
	public:
		remove_myself_node() :
			nh("~")
		{
			sub_pc = nh.subscribe("cloud_in", 1, &remove_myself_node::cb_cloud, this);
			pub_pc = nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 1, true);

			ang_res = M_PI / 180.0;
			width = M_PI * 2.0 / ang_res + 1;
			width_step = ((width + 31) / 32) * 32;

			nh.param("range", range, 0.5);
			nh.param("z_max", z_max, 100.0);
			nh.param("z_min", z_min, 0.15);
			nh.param("expand", expand, 2);
			nh.param("generate_mask", generate_mask, false);
			nh.param("mask_file", mask_file, std::string(""));

			if(!generate_mask)
			{
				ROS_INFO("Loading %s", mask_file.c_str());
				std::ifstream ifs(mask_file);
				if(ifs.fail())
				{
					ROS_ERROR("Failed to load mask_file");
					ros::shutdown();
					return;
				}
				while(true)
				{
					sensor s;
					s.mask.resize(width * width_step / 32);

					if(!getline(ifs, s.frame_id)) break;
					for(int i = 0; i < width; i ++)
					{
						std::string line;
						getline(ifs, line);
						for(int j = 0; j < width_step / 32; j ++)
						{
							int addr = i * width_step + j * 32;
							auto str = line.substr(j * 8, 8);
							s.mask[addr/32] = std::stoul(str, nullptr, 16);
						}
					}
					sensors.push_back(s);
				}
				ifs.close();
			}
			else
			{
				ROS_ERROR("Generate mask");
			}
		}
		void spin()
		{
			ros::spin();

			if(generate_mask)
			{
				std::ofstream ofs(mask_file);
				if(ofs.fail())
				{
					return;
				}
				for(auto &s: sensors)
				{
					ofs << s.frame_id << std::endl;
					for(int i = 0; i < width; i ++)
					{
						for(int j = 0; j < width_step / 32; j ++)
						{
							char str[8];
							int addr = i * width_step + j * 32;
							sprintf(str, "%08x", s.mask[addr/32]);
							ofs << str;
						}
						ofs << std::endl;
					}
				}
				ofs.close();
			}
		}
	private:
		void cb_cloud(const sensor_msgs::PointCloud2::Ptr &msg)
		{
			sensor *s = nullptr;
			for(auto &sen: sensors)
			{
				if(sen.frame_id.compare(msg->header.frame_id) == 0)
				{
					s = &sen;
					break;
				}
			}
			if(s == nullptr)
			{
				sensor news;
				news.frame_id = msg->header.frame_id;
				news.mask.resize(width * width_step / 32);
				sensors.push_back(news);
				s = &sensors.back();
				ROS_ERROR("New frame_id %s registered", s->frame_id.c_str());
			}
			
			sensor_msgs::PointCloud2 out;
			out.header = msg->header;
			out.is_dense = msg->is_dense;
			out.is_bigendian = msg->is_bigendian;
			out.fields = msg->fields;
			out.point_step = msg->point_step;
			out.height = 1;
			out.data.resize(msg->data.size());

			float *p = reinterpret_cast<float*>(&msg->data[0]);
			float *p2 = reinterpret_cast<float*>(&out.data[0]);
			int num = 0;
			for(unsigned int j = 0; j < msg->width; j ++)
			{
				const float &x = p[0];
				const float &y = p[1];
				const float &z = p[2];

				float r = hypotf(y, x);
				if(r < 0.3)
				{
					p += msg->point_step / sizeof(float);
					continue;
				}
				float yaw_raw = atan2(y, x);
				float pitch_raw = atan2(z, r);
				float yaw = yaw_raw;
				float pitch = pitch_raw;
				yaw += M_PI;
				pitch += M_PI;

				int iyaw = floorf(yaw / ang_res);
				int ipitch = floorf(pitch / ang_res);
				int addr = iyaw * width_step + ipitch;
				if(addr < 0 || width_step * width <= addr)
				{
					ROS_ERROR("Address error");
					continue;
				}
				uint32_t bit = 1 << (31 - (addr % 32));

				auto &tb = s->mask[addr / 32];

				if(generate_mask)
				{
					float r3 = sqrtf(powf(x, 2.0) + powf(y, 2.0) + powf(z, 2.0));
					if(r3 < range &&
							z_min < z && z < z_max)
					{
						for(int i = -expand; i <= expand; i ++)
						{
							for(int j = -expand; j <= expand; j ++)
							{
								int iyaw2 = iyaw + i;
								int ipitch2 = ipitch + j;
								if(iyaw2 < 0) iyaw2 += width;
								if(iyaw2 >= width) iyaw2 -= width;
								if(ipitch2 < 0) ipitch2 += width;
								if(ipitch2 >=width) ipitch2 -= width;

								int addr = iyaw2 * width_step + ipitch2;
								auto &tb = s->mask[addr / 32];
								uint32_t bit = 1 << (31 - (addr % 32));
								tb |= bit;
							}
						}
					}
				}

				if(!(tb & bit))
				{
					for(size_t i = 0; i < msg->point_step / sizeof(float); i ++)
					{
						p2[i] = p[i];
					}
					p2 += msg->point_step / sizeof(float);
					num ++;
				}

				p += msg->point_step / sizeof(float);
			}
			out.data.resize(num * out.point_step);
			out.width = num;
			out.row_step = num * out.point_step;
			pub_pc.publish(out);
		}
		ros::NodeHandle nh;
		ros::Publisher pub_pc;
		ros::Subscriber sub_pc;
		float ang_res;
		int width;
		int expand;
		int width_step;
		double range;
		double z_min;
		double z_max;
		bool generate_mask;
		std::string mask_file;

		class sensor{
		public:
			std::string frame_id;
			std::vector<uint32_t> mask;
		};
		std::vector<sensor> sensors;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "remove_myself");
	remove_myself_node node;

	node.spin();

	return 1;
}

