#include "slic.h"

/*
 * Constructor. Nothing is done here.
 */
Slic::Slic() {

}

/*
 * Destructor. Clear any present data.
 */
Slic::~Slic() {
    clear_data();
}

/*
 * Clear the data as saved by the algorithm.
 *
 * Input : -
 * Output: -
 */
void Slic::clear_data() {
    clusters.clear();
    distances.clear();
    centers.clear();
    center_counts.clear();
    colours.clear();
    modes.clear();
    indexmap.clear();
}

/*
 * Initialize the cluster centers and initial values of the pixel-wise cluster
 * assignment and distance values.
 *
 * Input : The image (IplImage*).
 * Output: -
 */
void Slic::init_data(IplImage *image) {
    /* Initialize the cluster and distance matrices. */
    for (int i = 0; i < image->width; i++) { 
        vector<int> cr;
        vector<double> dr;
        for (int j = 0; j < image->height; j++) {
            cr.push_back(-1);
            dr.push_back(FLT_MAX);
        }
        clusters.push_back(cr);
        distances.push_back(dr);
    }
    
    /* Initialize the centers and counters. */
    for (int i = step; i < image->width - step/2; i += step) {
        for (int j = step; j < image->height - step/2; j += step) {
            vector<double> center;
            /* Find the local minimum (gradient-wise). */
            CvPoint nc = find_local_minimum(image, cvPoint(i,j));
            CvScalar colour = cvGet2D(image, nc.y, nc.x);
            
            /* Generate the center vector. */
            center.push_back(colour.val[0]);
            center.push_back(colour.val[1]);
            center.push_back(colour.val[2]);
            center.push_back(nc.x);
            center.push_back(nc.y);
            
            /* Append to vector of centers. */
            centers.push_back(center);
            center_counts.push_back(0);
        }
    }

    /* Initialize colours */
    colours.resize(centers.size()); 
}

/*
 * Compute the distance between a cluster center and an individual pixel.
 *
 * Input : The cluster index (int), the pixel (CvPoint), and the Lab values of
 *         the pixel (CvScalar).
 * Output: The distance (double).
 */
double Slic::compute_dist(int ci, CvPoint pixel, CvScalar colour) {
    double dc = sqrt(pow(centers[ci][0] - colour.val[0], 2) + pow(centers[ci][1]
            - colour.val[1], 2) + pow(centers[ci][2] - colour.val[2], 2));
    double ds = sqrt(pow(centers[ci][3] - pixel.x, 2) + pow(centers[ci][4] - pixel.y, 2));
    
    return sqrt(pow(dc / nc, 2) + pow(ds / ns, 2));
    
    //double w = 1.0 / (pow(ns / nc, 2));
    //return sqrt(dc) + sqrt(ds * w);
}

/*
 * Find a local gradient minimum of a pixel in a 3x3 neighbourhood. This
 * method is called upon initialization of the cluster centers.
 *
 * Input : The image (IplImage*) and the pixel center (CvPoint).
 * Output: The local gradient minimum (CvPoint).
 */
CvPoint Slic::find_local_minimum(IplImage *image, CvPoint center) {
    double min_grad = FLT_MAX;
    CvPoint loc_min = cvPoint(center.x, center.y);
    
    for (int i = center.x-1; i < center.x+2; i++) {
        for (int j = center.y-1; j < center.y+2; j++) {
            CvScalar c1 = cvGet2D(image, j+1, i);
            CvScalar c2 = cvGet2D(image, j, i+1);
            CvScalar c3 = cvGet2D(image, j, i);
            /* Convert colour values to grayscale values. */
            double i1 = c1.val[0];
            double i2 = c2.val[0];
            double i3 = c3.val[0];
            /*double i1 = c1.val[0] * 0.11 + c1.val[1] * 0.59 + c1.val[2] * 0.3;
            double i2 = c2.val[0] * 0.11 + c2.val[1] * 0.59 + c2.val[2] * 0.3;
            double i3 = c3.val[0] * 0.11 + c3.val[1] * 0.59 + c3.val[2] * 0.3;*/
            
            /* Compute horizontal and vertical gradients and keep track of the
               minimum. */
            if (sqrt(pow(i1 - i3, 2)) + sqrt(pow(i2 - i3,2)) < min_grad) {
                min_grad = fabs(i1 - i3) + fabs(i2 - i3);
                loc_min.x = i;
                loc_min.y = j;
            }
        }
    }
    
    return loc_min;
}

/*
 * Compute the over-segmentation based on the step-size and relative weighting
 * of the pixel and colour values.
 *
 * Input : The Lab image (IplImage*), the stepsize (int), and the weight (int).
 * Output: -
 */
void Slic::generate_superpixels(IplImage *image, int step, int nc) {
    this->step = step;
    this->nc = nc;
    this->ns = step;
    
    /* Clear previous data (if any), and re-initialize it. */
    clear_data();
    init_data(image);
    
    /* Run EM for 10 iterations (as prescribed by the algorithm). */
    for (int i = 0; i < NR_ITERATIONS; i++) {
        /* Reset distance values. */
        for (int j = 0; j < image->width; j++) {
            for (int k = 0;k < image->height; k++) {
                distances[j][k] = FLT_MAX;
            }
        }

        for (int j = 0; j < (int) centers.size(); j++) {
            /* Only compare to pixels in a 2 x step by 2 x step region. */
            for (int k = centers[j][3] - step; k < centers[j][3] + step; k++) {
                for (int l = centers[j][4] - step; l < centers[j][4] + step; l++) {
                
                    if (k >= 0 && k < image->width && l >= 0 && l < image->height) {
                        CvScalar colour = cvGet2D(image, l, k);
                        double d = compute_dist(j, cvPoint(k,l), colour);
                        
                        /* Update cluster allocation if the cluster minimizes the
                           distance. */
                        if (d < distances[k][l]) {
                            distances[k][l] = d;
                            clusters[k][l] = j;
                        }
                    }
                }
            }
        }
        
        /* Clear the center values. */
        for (int j = 0; j < (int) centers.size(); j++) {
            centers[j][0] = centers[j][1] = centers[j][2] = centers[j][3] = centers[j][4] = 0;
            center_counts[j] = 0;
        }
        
        /* Compute the new cluster centers. */
        for (int j = 0; j < image->width; j++) {
            for (int k = 0; k < image->height; k++) {
                int c_id = clusters[j][k];
                
                if (c_id != -1) {
                    CvScalar colour = cvGet2D(image, k, j);
                    
                    centers[c_id][0] += colour.val[0];
                    centers[c_id][1] += colour.val[1];
                    centers[c_id][2] += colour.val[2];
                    centers[c_id][3] += j;
                    centers[c_id][4] += k;
                    
                    center_counts[c_id] += 1;
                }
            }
        }

        /* Normalize the clusters. */
        for (int j = 0; j < (int) centers.size(); j++) {
            centers[j][0] /= center_counts[j];
            centers[j][1] /= center_counts[j];
            centers[j][2] /= center_counts[j];
            centers[j][3] /= center_counts[j];
            centers[j][4] /= center_counts[j];
        }
    }
}

/*
 * Enforce connectivity of the superpixels. This part is not actively discussed
 * in the paper, but forms an active part of the implementation of the authors
 * of the paper.
 *
 * Input : The image (IplImage*).
 * Output: -
 */
void Slic::create_connectivity(IplImage *image) {
    int label = 0, adjlabel = 0;
    const int lims = (image->width * image->height) / ((int)centers.size());
    
    const int dx4[4] = {-1,  0,  1,  0};
	const int dy4[4] = { 0, -1,  0,  1};
    
    /* Initialize the new cluster matrix. */
    vec2di new_clusters;
    for (int i = 0; i < image->width; i++) { 
        vector<int> nc;
        for (int j = 0; j < image->height; j++) {
            nc.push_back(-1);
        }
        new_clusters.push_back(nc);
    }

    for (int i = 0; i < image->width; i++) {
        for (int j = 0; j < image->height; j++) {
            if (new_clusters[i][j] == -1) {
                vector<CvPoint> elements;
                elements.push_back(cvPoint(i, j));
            
                /* Find an adjacent label, for possible use later. */
                for (int k = 0; k < 4; k++) {
                    int x = elements[0].x + dx4[k], y = elements[0].y + dy4[k];
                    
                    if (x >= 0 && x < image->width && y >= 0 && y < image->height) {
                        if (new_clusters[x][y] >= 0) {
                            adjlabel = new_clusters[x][y];
                        }
                    }
                }
                
                int count = 1;
                for (int c = 0; c < count; c++) {
                    for (int k = 0; k < 4; k++) {
                        int x = elements[c].x + dx4[k], y = elements[c].y + dy4[k];
                        
                        if (x >= 0 && x < image->width && y >= 0 && y < image->height) {
                            if (new_clusters[x][y] == -1 && clusters[i][j] == clusters[x][y]) {
                                elements.push_back(cvPoint(x, y));
                                new_clusters[x][y] = label;
                                count += 1;
                            }
                        }
                    }
                }
                
                /* Use the earlier found adjacent label if a segment size is
                   smaller than a limit. */
                if (count <= lims >> 2) {
                    for (int c = 0; c < count; c++) {
                        new_clusters[elements[c].x][elements[c].y] = adjlabel;
                    }
                    label -= 1;
                }
                label += 1;
            }
        }
    }
}

/*
 * Display the cluster centers.
 *
 * Input : The image to display upon (IplImage*) and the colour (CvScalar).
 * Output: -
 */
void Slic::display_center_grid(IplImage *image, CvScalar colour) {
    for (int i = 0; i < (int) centers.size(); i++) {
        cvCircle(image, cvPoint(centers[i][3], centers[i][4]), 2, colour, 2);
    }
}

/*
 * Display a single pixel wide contour around the clusters.
 *
 * Input : The target image (IplImage*) and contour colour (CvScalar).
 * Output: -
 */
void Slic::display_contours(IplImage *image, CvScalar colour) {
    const int dx8[8] = {-1, -1,  0,  1, 1, 1, 0, -1};
	const int dy8[8] = { 0, -1, -1, -1, 0, 1, 1,  1};
	
	/* Initialize the contour vector and the matrix detailing whether a pixel
	 * is already taken to be a contour. */
	vector<CvPoint> contours;
	vec2db istaken;
	for (int i = 0; i < image->width; i++) { 
        vector<bool> nb;
        for (int j = 0; j < image->height; j++) {
            nb.push_back(false);
        }
        istaken.push_back(nb);
    }
    
    /* Go through all the pixels. */
    for (int i = 0; i < image->width; i++) {
        for (int j = 0; j < image->height; j++) {
            int nr_p = 0;
            
            /* Compare the pixel to its 8 neighbours. */
            for (int k = 0; k < 8; k++) {
                int x = i + dx8[k], y = j + dy8[k];
                
                if (x >= 0 && x < image->width && y >= 0 && y < image->height) {
                    if (istaken[x][y] == false && clusters[i][j] != clusters[x][y]) {
                        nr_p += 1;
                    }
                }
            }
            
            /* Add the pixel to the contour list if desired. */
            if (nr_p >= 2) {
                contours.push_back(cvPoint(i,j));
                istaken[i][j] = true;
            }
        }
    }
    
    /* Draw the contour pixels. */
    for (int i = 0; i < (int)contours.size(); i++) {
        cvSet2D(image, contours[i].y, contours[i].x, colour);
    }
}

/*
 * Computes the mean colours. It changes the colours attribute
 *
 * Input: The target image (IplImage*)
 * Output: -
 */
void Slic::compute_cluster_means(IplImage *image) {
    /* Gather the colour values per cluster. */
    for (int i = 0; i < image->width; i++) {
        for (int j = 0; j < image->height; j++) {
            int index = clusters[i][j];
            CvScalar colour = cvGet2D(image, j, i);
            
            colours[index].val[0] += colour.val[0];
            colours[index].val[1] += colour.val[1];
            colours[index].val[2] += colour.val[2];
        }
    }
    
    /* Divide by the number of pixels per cluster to get the mean colour. */
    for (int i = 0; i < (int)colours.size(); i++) {
        colours[i].val[0] /= center_counts[i];
        colours[i].val[1] /= center_counts[i];
        colours[i].val[2] /= center_counts[i];
    }
}

/*
 * Give the pixels of each cluster the same colour values. The specified colour
 * is the mean RGB colour per cluster.
 *
 * Input : The target image (IplImage*).
 * Output: -
 */
void Slic::colour_with_cluster_means(IplImage *image) {
    
    Slic::compute_cluster_means(image);

    /* Fill in. */
    for (int i = 0; i < image->width; i++) {
        for (int j = 0; j < image->height; j++) {
            CvScalar ncolour = colours[clusters[i][j]];
            cvSet2D(image, j, i, ncolour);
        }
    }
}

double Slic::mu(vector<double> v) { //Mean
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();
    return mean;
}

double Slic::std(vector<double> v) { //Standard deviation
    double mean = Slic::mu(v);
    vector<double> diff(v.size());
    transform(v.begin(), v.end(), diff.begin(),
		   bind2nd(minus<double>(), mean));
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = sqrt(sq_sum / v.size()); 
    return stdev;
}

void Slic::two_level_cluster(IplImage *image, CvScalar template_color, int kernel_type, double kernel_bandwidth , int dim, double mode_tolerance) {

    /* Similar to compute cluster means, but is saving some values for zscoring */ 

    //
    // Compute Mean Color of Each Super Pixel
    // 
    // Sum

    for (int i = 0; i < image->width; i++) {
        for (int j = 0; j < image->height; j++) {
            int index = clusters[i][j];
            CvScalar colour = cvGet2D(image, j, i);
            
            colours[index].val[0] += colour.val[0];

            colours[index].val[1] += colour.val[1];
            colours[index].val[2] += colour.val[2];
        }
    }

    // Average
    vector<double> channel1(colours.size());
    vector<double> channel2(colours.size());
    vector<double> channel3(colours.size());
    /* Divide by the number of pixels per cluster to get the mean colour. */
    for (int i = 0; i < (int)colours.size(); i++) {
        colours[i].val[0] /= center_counts[i];
        colours[i].val[1] /= center_counts[i];
        colours[i].val[2] /= center_counts[i];
	channel1[i] = colours[i].val[0];
	channel2[i] = colours[i].val[1];
	channel3[i] = colours[i].val[2];
    }

    //
    // Compute Normalized Value (z-score) for better clustering
    //
    double channel1_mu = Slic::mu(channel1);
    double channel1_std = Slic::std(channel1);
    double channel2_mu = Slic::mu(channel2);
    double channel2_std = Slic::std(channel2);
    double channel3_mu = Slic::mu(channel3);
    double channel3_std = Slic::std(channel3);
    /* Convert into data format and zscore for Meanshift */ 
    vec2dd X;

    for (int i = 0; i < (int)colours.size(); i++) {

        vector<double> data_point;
        data_point.push_back((colours[i].val[0] - channel1_mu)/channel1_std);
        data_point.push_back((colours[i].val[1] - channel2_mu)/channel2_std);
        data_point.push_back((colours[i].val[2] - channel3_mu)/channel3_std);

        X.push_back(data_point);
    }
   
    //
    // Meanshift clustering of average color of each super pixels, basically making groups of superpixels
    //
    clustering::Meanshift estimator(kernel_type, kernel_bandwidth, dim, mode_tolerance);
    estimator.FindModes(X, modes, indexmap, 0);
}

CvScalar Slic::calibrate_template_color(IplImage* image, IplImage* depth_channel) {
   
    cv::Mat depth_channel_mat(depth_channel); 
    int lidar_row_index = static_cast<int>(0.33*depth_channel->height);
    /* cv::Mat lidar_row = depth_channel_mat.row(lidar_row_index); */
    /* const unsigned char* lidar_row = depth_channel_mat.ptr<unsigned char>(lidar_row_index); */
  
    int mode_idx;
    vec2di modes_to_lidar(modes.size()); 
    for (int col = 0; col < depth_channel->width; ++col) {
        // construct arrays mapping depths to modes; preparing to average over depths for each mode
        int temp2 = clusters[col][lidar_row_index];

        mode_idx = indexmap[temp2];
        /* modes_to_lidar[mode_idx].push_back(depth_channel_mat.at(lidar_row_index, col)); */
        /* modes_to_lidar[mode_idx].push_back(lidar_row[col]); */

        // float color = small_hue.at<float>(indexrow,indexcol, 0);
        int temp = depth_channel_mat.at<int>(lidar_row_index, col);

        if (temp < 0) {
            std::cout << "temp < 0!" << std::endl;
        }
        modes_to_lidar[mode_idx].push_back(temp);
    } 

    // Finds the average depth per mode of clusters.
    vector<double> avg_depth_per_mode(modes.size());
    for (int mode_idx = 0; mode_idx < (int)modes.size(); mode_idx++) {
        // compute average over array of depths for this mode
        
        int avg_depth;

        if (modes_to_lidar[mode_idx].size() != 0){
            // avg_depth = std::accumulate(modes_to_lidar[mode_idx].begin(),
            //                             modes_to_lidar[mode_idx].end(),
            //                                0LL) / modes_to_lidar[mode_idx].size();

            std::sort(modes_to_lidar[mode_idx].begin(), modes_to_lidar[mode_idx].end());
            
            avg_depth = modes_to_lidar[mode_idx][modes_to_lidar[mode_idx].size()/2];

        }
        else{
            avg_depth = 0;
        }
        
                        
        // assign average
        avg_depth_per_mode[mode_idx] = avg_depth;

        std::cout << "Avg Depth: " << avg_depth << std::endl;
    }


    int our_mode = std::max_element(avg_depth_per_mode.begin(), avg_depth_per_mode.end()) - avg_depth_per_mode.begin();

    // Color 1/3 for testing purposes 
    CvScalar cvWhite = {{255,255,255}};
    for (int column = 0; column < image->width; ++column) {
        cvSet2D(image, lidar_row_index, column, cvWhite);
    } 

    // Color mode for validating
    /* Fill in. */
    for (int i = 0; i < image->width; i++) {
        for (int j = 0; j < image->height; j++) {
            if (our_mode == indexmap[clusters[i][j]]) {
                cvSet2D(image, j, i, cvWhite);
            }
        }
     } 
   
    return cvWhite; 
    /* // Get Mean of the Modes */
    /* vector<CvScalar> mode_colours(modes.size()); */
    /* vector<int> mode_counts(modes.size()); */
    /* for (int i = 0; i < (int)colours.size(); ++i) { */
    /*     mode_colours[indexmap[i]].val[0] += colours[i].val[0]; */
    /*     mode_colours[indexmap[i]].val[1] += colours[i].val[1]; */
    /*     mode_colours[indexmap[i]].val[2] += colours[i].val[2]; */
    /*     mode_counts[indexmap[i]] += 1; */
    /* } */ 
    /* for (int i = 0; i < (int)mode_colours.size(); ++i) { */
    /*     mode_colours[i].val[0] /= mode_counts[i]; */
    /*     mode_colours[i].val[1] /= mode_counts[i]; */
    /*     mode_colours[i].val[2] /= mode_counts[i]; */
    /* } */ 

}
/*
 * Based on a template color, it returns a mask image which has the cluster of interest white
 *
 * Input
 * - image: to be transformed into black and white
 * - template_color: color we are selecting for
 */
/* void compare_template_color(IplImage* image, cvScalar template_color) { */
/*     // Get Mean of the Modes */
/*     vector<CvScalar> mode_colours(modes.size()); */
/*     vector<int> mode_counts(modes.size()); */
/*     for (int i = 0; i < (int)colours.size(); ++i) { */
/*         mode_colours[indexmap[i]].val[0] += colours[i].val[0]; */
/*         mode_colours[indexmap[i]].val[1] += colours[i].val[1]; */
/*         mode_colours[indexmap[i]].val[2] += colours[i].val[2]; */
/*         mode_counts[indexmap[i]] += 1; */
/*     } */ 
/*     for (int i = 0; i < (int)mode_colours.size(); ++i) { */
/*         mode_colours[i].val[0] /= mode_counts[i]; */
/*         mode_colours[i].val[1] /= mode_counts[i]; */
/*         mode_colours[i].val[2] /= mode_counts[i]; */
/*     } */ 

/*     vector<double> colorspace_dist(mode_colours.size()); */
/*     for (int i = 0; i < (int)mode_colours.size(); ++i) { */
/*         colorspace_dist[i] = sqrt(pow(template_color.val[0]-mode_colours[i].val[0],2)+ */
/*                                   pow(template_color.val[1]-mode_colours[i].val[1],2)); */
/*                                   /1* pow(template_color.val[2]-mode_colours[i].val[2],2)); *1/ */
/*     } */
/*     int our_mode = std::distance(colorspace_dist.begin(), std::min_element(colorspace_dist.begin(), colorspace_dist.end())); */

/*     CvScalar cvWhite = {{255,255,255}}; */
/*     CvScalar cvBlack = {{0,0,0}}; */
/*     /1* Fill in. *1/ */
/*     for (int i = 0; i < image->width; i++) { */
/*         for (int j = 0; j < image->height; j++) { */
/*             if (our_mode == indexmap[clusters[i][j]]) { */
/*                 cvSet2D(image, j, i, cvWhite); */
/*             } else { */
/*                 cvSet2D(image, j, i, cvBlack); */
/*             } */
/*         } */
/*     } */
/* } */
