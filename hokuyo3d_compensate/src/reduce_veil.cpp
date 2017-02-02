#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class reduce_veil_node
{
	public:
		reduce_veil_node() :
			nh("~")
		{
			sub_pc = nh.subscribe("cloud_in", 1, &reduce_veil_node::cb_cloud, this);
			pub_pc = nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 1, true);
			nh.param("ang_lim", ang_lim, 0.1);
		}
	private:
		double ang_lim;
		void cb_cloud(const sensor_msgs::PointCloud2::Ptr &msg)
		{
			std::vector<bool> mask;
			mask.resize(msg->width, true);
			float *data = reinterpret_cast<float*>(&msg->data[0]);
			float x_prev;
			float y_prev;
			float z_prev;
			for(unsigned int j = 0; j < msg->width; j ++)
			{
				float x = *(data++);
				float y = *(data++);
				float z = *(data++);
				//float i = *(data++);
				data ++;

				if(j > 0)
				{
					float v0[3] = {x, y, z};
					float v1[3] = {x - x_prev, y - y_prev, z - z_prev};
					float r0 = sqrtf(powf(v0[0], 2) + powf(v0[1], 2) + powf(v0[2], 2));
					float r1 = sqrtf(powf(v1[0], 2) + powf(v1[1], 2) + powf(v1[2], 2));
					float dot = v0[0] * v1[0] + v0[1] * v1[1] + v0[2] * v1[2];
					float ang = acosf(dot / (r0 * r1));
					if(ang > M_PI / 2.0) ang = M_PI - ang;
					if(fabs(ang) < ang_lim)
					{
						mask[j] = false;
						mask[j-1] = false;
					}
				}
				else
				{
					mask[j] = false;
				}

				x_prev = x;
				y_prev = y;
				z_prev = z;
			}
			sensor_msgs::PointCloud2 pc;
			pc.is_bigendian = msg->is_bigendian;
			pc.fields = msg->fields;
			pc.point_step = msg->point_step;
			pc.is_dense = true;
			pc.row_step = msg->row_step;
			pc.header = msg->header;
			pc.width = 0;
			pc.height = 1;
			pc.data.resize(msg->width * pc.point_step);
			float *data2 = reinterpret_cast<float*>(&pc.data[0]);
			data = reinterpret_cast<float*>(&msg->data[0]);
			for(unsigned int j = 0; j < msg->width; j ++)
			{
				if(!mask[j])
				{
					data += 4;
					continue;
				}
				*(data2++) = *(data++);
				*(data2++) = *(data++);
				*(data2++) = *(data++);
				*(data2++) = *(data++);
				pc.width ++;
			}
			pc.data.resize(pc.width * pc.point_step);
			pub_pc.publish(pc);
		}
		ros::NodeHandle nh;
		ros::Publisher pub_pc;
		ros::Subscriber sub_pc;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "reduce_veil");
	reduce_veil_node node;

	ros::spin();

	return 1;
}

