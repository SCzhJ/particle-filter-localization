#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

class LikelihoodFieldGenerator
{
public:
    LikelihoodFieldGenerator()
    {
        // Initialize ROS node
        ros::NodeHandle nh;

        // Wait for OccupancyGrid message
        nav_msgs::OccupancyGridConstPtr msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", nh);

        if (msg != nullptr)
        {
            this->width = msg->info.width;
            this->height = msg->info.height;
            this->resolution = msg->info.resolution;
            // Generate likelihood field
            generateLikelihoodField(msg);
            readLikelihoodFieldAndPublish();
            this->pub = nh.advertise<nav_msgs::OccupancyGrid>("/likelihood_field", 1);
            ROS_INFO("Publishing likelihood field");
            this->pub.publish(this->likelihoodfield_msg);
        }
        else
        {
            ROS_ERROR("No message received on /map");
        }
    }

    void generateLikelihoodField(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        // Create a point cloud to store the occupied cells
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Iterate over the OccupancyGrid
        for (int i = 0; i < msg->info.height; i++)
        {
            for (int j = 0; j < msg->info.width; j++)
            {
                // If the cell is occupied, add it to the point cloud
                if (msg->data[i * msg->info.width + j] > 0)
                {
                    pcl::PointXYZ point;
                    point.x = j * msg->info.resolution;
                    point.y = i * msg->info.resolution;
                    point.z = 0;
                    cloud->points.push_back(point);
                }
            }
        }

        // Create a KD Tree
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);

        // Create a likelihood field with the same dimensions as the OccupancyGrid
        std::vector<float> likelihood_field(msg->info.width * msg->info.height);

        // Iterate over the OccupancyGrid again
        for (int i = 0; i < msg->info.height; i++)
        {
            for (int j = 0; j < msg->info.width; j++)
            {
                pcl::PointXYZ searchPoint;
                searchPoint.x = j * msg->info.resolution;
                searchPoint.y = i * msg->info.resolution;
                searchPoint.z = 0;

                // Find the nearest occupied cell
                std::vector<int> pointIdxNKNSearch(1);
                std::vector<float> pointNKNSquaredDistance(1);
                if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
                {
                    // Calculate the Gaussian probability based on the distance
                    float sigma = 0.5; 
                    float mu = 0; 
                    float probability = exp(-pow(pointNKNSquaredDistance[0] - mu, 2) / (2 * pow(sigma, 2))) / (sigma * sqrt(2 * M_PI));
                    likelihood_field[i * msg->info.width + j] = probability;
                }
            }
        }

        // After the loop that generates the likelihood field
        std::ofstream file("likelihood_field.txt");
        if (file.is_open())
        {
            for (const auto& probability : likelihood_field)
            {
                file << probability << "\n";
            }
            file.close();
        }
        else
        {
            ROS_ERROR("Unable to open file");
        }

    }

    void readLikelihoodFieldAndPublish(){
        // Read likelihood field from file
        std::ifstream file("likelihood_field.txt");
        if (file.is_open())
        {
            float probability;
            while (file >> probability)
            {
                this->likelihood_field.push_back(probability);
            }
            file.close();

            this->likelihoodfield_msg.header.stamp = ros::Time::now();
            this->likelihoodfield_msg.header.frame_id = "likelihoodfield";
            this->likelihoodfield_msg.info.resolution = this->resolution;
            this->likelihoodfield_msg.info.width = this->width;
            this->likelihoodfield_msg.info.height = this->height;
            this->likelihoodfield_msg.info.origin.position.x = 0;
            this->likelihoodfield_msg.info.origin.position.y = 0;
            this->likelihoodfield_msg.info.origin.position.z = 0;
            this->likelihoodfield_msg.info.origin.orientation.x = 0;
            this->likelihoodfield_msg.info.origin.orientation.y = 0;
            this->likelihoodfield_msg.info.origin.orientation.z = 0;
            this->likelihoodfield_msg.info.origin.orientation.w = 1;
            this->likelihoodfield_msg.data.resize(this->width * this->height);

            // Fill the OccupancyGrid with the likelihood field
            for (int i = 0; i < this->height; i++)
            {
                for (int j = 0; j < this->width; j++)
                {
                    this->likelihoodfield_msg.data[i * this->width + j] = this->likelihood_field[i * this->width + j] * 100;
                }
            }
        }
        else
        {
            ROS_ERROR("Unable to open file");
        }
    }

private:
    ros::Publisher pub;
    std::vector<float> likelihood_field;
    nav_msgs::OccupancyGrid likelihoodfield_msg;
    float width;float height;float resolution;
};

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "likelihood_field_generator");

    // Create likelihood field generator
    LikelihoodFieldGenerator lfg;

    ros::spin();
    return 0;
}    