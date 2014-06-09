//CImg for image access
#include "CImg.h"

//Standard includes
#include <math.h>
#include <iostream>
#include <vector>

//My Quaternion class
#include "quaternion.h"

//Get the x and y
int gx[3][3] = { {-1, 0, 1},{-2, 0, 2},{-1, 0, 1} };
int gy[3][3] = { {-1, -2, -1},{0, 0, 0},{1, 2, 1} };

//Get the distance in colorspace between Quaternions
float distance(Quaternion q1, Quaternion q2, float theta) {
    Quaternion u(0, 1/sqrt(3), 1/sqrt(3), 1/sqrt(3));
    Quaternion U(cos(theta), sin(theta)/sqrt(3), sin(theta)/sqrt(3), sin(theta)/sqrt(3));
    Quaternion Y[] = { (U*q1*(U.conjugate())), (U*q2*(U.conjugate())) };
    Quaternion Yp[] = { ((U.conjugate())*q1*U), ((U.conjugate())*q2*U) };

    float intensity = (Y[0]+Yp[0]-(Y[1]+Yp[1])).modulus() * 0.5;
    float chromacity = (Y[0]-Yp[0]-(Y[1]-Yp[1])).modulus() * 0.5;

    return(intensity+chromacity);
}

//Weighted for the filtered average
float weight(Quaternion q, Quaternion center, float theta) {
    return(exp(-distance(q, center, theta)));
}

//Quaternion Weighted Average Filter
Quaternion QWAF(Quaternion q[9]) {
    Quaternion numerator;
    float denominator = 0;

    //Loop through all of that
    for (int i=0; i<9; i++) {
        float dir = 0;

        switch(i) {
            case 0: dir = 3*M_PI/4; break;
            case 1: dir = M_PI/2; break;
            case 2: dir = M_PI/4; break;
            case 3: dir = M_PI; break;
            case 4: dir = 0; break;
            case 5: dir = 0; break;
            case 6: dir = 5*M_PI/4; break;
            case 7: dir = 3*M_PI/2; break;
            case 8: dir = 7*M_PI/4; break;
        }

        float qw = weight(q[i], q[4], dir);
        numerator += q[i]*qw;
        denominator += qw;
    }

    return(numerator / denominator);
}

//HSV to RGB
void HSVtoRGB( float *r, float *g, float *b, float h, float s, float v )
{
    int i;
	float f, p, q, t;
	if( s == 0 ) { *r = *g = *b = v; return; }
	h /= 60; i = (int)floor( h ); f = h - i;
	p = v * (1-s); q = v * (1-s*f);	t = v * (1-s*(1-f));
	switch( i ) {
		case 0:	*r = v;	*g = t;	*b = p;	break;
		case 1:	*r = q; *g = v;	*b = p;	break;
		case 2:	*r = p;	*g = v;	*b = t;	break;
		case 3:	*r = p;	*g = q;	*b = v;	break;
		case 4:	*r = t;	*g = p;	*b = v;	break;
		default:*r = v;	*g = p;	*b = q;	break;
	}
}

template<class T> T greyVal(const cimg_library::CImg<T>& image, int x, int y) {
    return((T)(0.299*(float)image(x,y,0,0)+0.587*(float)image(x,y,0,1)+0.114*(float)image(x,y,0,2)));
}

template<class T>
void sobelOperator(const cimg_library::CImg<T>& image, std::vector<T>& magnitude, std::vector<float>& direction) {
    //Allocate the magnitude
    magnitude.resize(image.width()*image.height());
    direction.resize(image.width()*image.height());

    //Loop through the image and apply the sobel operator
    for (int x=0; x<image.width(); x++) {
        for (int y=0; y<image.height(); y++) {
            int dx=0, dy=0, mag = 0;
            float dir=0;

            if (!(x == 0 || x == image.width()-1 || y == 0 || y == image.height()-1)) {
                for (int i=-1; i<=1; i++) {
                    for (int j=-1; j<=1; j++) {
                        dx = dx + gx[j+1][i+1] * greyVal(image, x+j, y+i);
                        dy = dy + gy[j+1][i+1] * greyVal(image, x+j, y+i);
                    }
                }
            }

            mag = (int)sqrt(pow((double)dx,2) + pow((double)dy,2));
            dir = atan2(dy, dx);

            if (mag > 255) mag = 255;
            if (mag < 0) mag = 0;

            magnitude[y*image.width()+x] = mag;
            direction[y*image.width()+x] = dir;
        }
    }
}

//Write the magnitude to the file
template<class T>
void writeMagnitude(cimg_library::CImg<T>& image, const std::vector<T>& magnitude) {
    for (int x=0; x<image.width(); x++) {
        for (int y=0; y<image.height(); y++) {
            const unsigned char mag = magnitude[y*image.width()+x];
            const unsigned char color_mag[] = { mag, mag, mag };
            image.draw_point(x, y, color_mag);
        }
    }
}

//Write the magnitude to the file
template<class T>
void writeMagnitudeDirection(cimg_library::CImg<T>& image, const std::vector<T>& magnitude, const std::vector<float>& direction) {
    for (int x=0; x<image.width(); x++) {
        for (int y=0; y<image.height(); y++) {
            float _r,_g,_b;
            HSVtoRGB(&_r, &_g, &_b, direction[y*image.width()+x]*180/M_PI, 1, (float)magnitude[y*image.width()+x]/255.0);
            const T color_dir[] = { (int)floor(_r*255), (int)floor(_g*255), (int)floor(_b*255) };
            image.draw_point(x, y, color_dir);
        }
    }
}

//Non-maxima Supression
template<class T>
void NMS(std::vector<T>& nms, const std::vector<T>& magnitude, const std::vector<float>& direction, int width, int height) {
    //Prepare the nms vector
    nms.resize(magnitude.size());

    //Loop through and do it
    for (int x=0; x<width; x++) {
        for (int y=0; y<height; y++) {
            T mag = magnitude[width*y+x];
            int dir = (int)floor((direction[width*y+x]+22.5) / 45.f) % 8;
            T grad[] = { 0, mag, 0 };

            //Assign the negative and positive points along the gradient
            int ng[2], pg[2];
            if (dir == 0 || dir == 4) { ng[0] = x-1; ng[1] = y;   pg[0] = x+1; pg[1] = y; }
            if (dir == 1 || dir == 5) { ng[0] = x-1; ng[1] = y-1; pg[0] = x+1; pg[1] = y+1; }
            if (dir == 2 || dir == 6) { ng[0] = x;   ng[1] = y-1; pg[0] = x;   pg[1] = y+1; }
            if (dir == 3 || dir == 7) { ng[0] = x+1; ng[1] = y+1; pg[0] = x-1; pg[1] = y-1; }

            //Limit absolute positions keeping them within the bounds of the image
            if (ng[0] < 0) ng[0] = 0;   if (ng[1] < 0) ng[1] = 0;
            if (pg[0] < 0) pg[0] = 0;   if (pg[1] < 0) pg[1] = 0;
            if (ng[0] >= width) ng[0] = width-1;   if (ng[1] >= height) ng[1] = height-1;
            if (pg[0] >= width) pg[0] = width-1;   if (pg[1] >= height) ng[1] = height-1;

            //Set the gradients
            grad[0] = magnitude[width*(ng[1])+(ng[0])];
            grad[2] = magnitude[width*(pg[1])+(pg[0])];

            //Assign the magnitude array
            nms[width*y+x] = (grad[1] >= grad[0] && grad[1] >= grad[2]) ? grad[1] : 0;
        }
    }
}

int cl(const int val, const int max) {
    int ret = val;
    if (ret < 0) ret = 0;
    if (ret >= max) ret = max-1;
    return(ret);
}

int coord(int& x, int& y, int width, int height) {
    x = cl(x, width);
    y = cl(y, height);
    return(width*y+x);
}

//Appy the Quaternion Image Filter
template<class T>
void applyQWAF(cimg_library::CImg<T>& image,cimg_library::CImg<T>& out) {
    for (int x=0; x<image.width(); x++) {
        for (int y=0; y<image.height(); y++) {
            Quaternion q[9];
            for (int ix=-1; ix<=1; ix++) {
                for (int iy=-1; iy<=1; iy++) {
                    int mx=x,my=y;
                    coord(mx, my, image.width(), image.height());
                    q[(iy+1)*3+(ix+1)] = Quaternion(0,image(mx,my,0,0),image(mx,my,0,1),image(mx,my,0,2));
                }
            }

            //Get the weighted average filter
            Quaternion p = QWAF(q);
            int mx=x, my=y;
            coord(mx, my, image.width(), image.height());
            T col[3] = { (T)p.b, (T)p.c, (T)p.d };
            out.draw_point(mx, my, col);
        }
    }
}

template <class T>
void sobelOperatorColor(const cimg_library::CImg<T>& image, std::vector<T>& magnitude, std::vector<float>& direction) {
    //Allocate the magnitude
    magnitude.resize(image.width()*image.height());
    direction.resize(image.width()*image.height());

    //Loop through the image and apply the sobel operator
    for (int x=0; x<image.width(); x++) {
        for (int y=0; y<image.height(); y++) {
            int dx=0, dy=0, mag = 0;
            float dir=0;

            if (!(x == 0 || x == image.width()-1 || y == 0 || y == image.height()-1)) {
                for (int i=-1; i<=1; i++) {
                    for (int j=-1; j<=1; j++) {
                        dx = dx + gx[j+1][i+1] * greyVal(image, x+j, y+i);
                        dy = dy + gy[j+1][i+1] * greyVal(image, x+j, y+i);
                    }
                }
            }

            mag = (int)sqrt(pow((double)dx,2) + pow((double)dy,2));
            dir = atan2(dy, dx);

            if (mag > 255) mag = 255;
            if (mag < 0) mag = 0;

            magnitude[y*image.width()+x] = mag;
            direction[y*image.width()+x] = dir;
        }
    }
}

//Main function
int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cout << "Usage: colorcanny image" << std::endl;
        return(0);
    }

    //CImg for output
    cimg_library::CImg <unsigned char> image(argv[1]);
    cimg_library::CImg <unsigned char> image_copy(image.width(), image.height(), image.depth(), image.spectrum());
    cimg_library::CImg <unsigned char> image_qwaf(image.width(), image.height(), image.depth(), image.spectrum());
    cimg_library::CImg <unsigned char> sobel(image.width(), image.height(), image.depth(), image.spectrum());
    cimg_library::CImg <unsigned char> sobel_dir(image.width(), image.height(), image.depth(), image.spectrum());
    cimg_library::CImg <unsigned char> nms_image(image.width(), image.height(), image.depth(), image.spectrum());
    cimg_library::CImg <unsigned char> nms_image_dir(image.width(), image.height(), image.depth(), image.spectrum());

    //Do a copy for testing
    for (int x=0; x<image.width(); x++) {
        for (int y=0; y<image.height(); y++) {
            unsigned char col[] = {image(x,y,0,0), image(x,y,0,1), image(x,y,0,2)};
            image_copy.draw_point(x,y,col);
        }
    }

    applyQWAF(image, image_qwaf);
    image_qwaf.save("image_qwaf.bmp");

    //Copy to see if everything is working properly
    image_copy.save("image_copy.bmp");

    //Apply a gaussian blur
    image.blur(1.6);

    //Vectors
    std::vector<unsigned char> magnitude;
    std::vector<unsigned char> nms;
    std::vector<float> direction;

    //Apply the sobel operator to the image and then non-maximum supression
    sobelOperator(image, magnitude, direction);
    NMS(nms, magnitude, direction, image.width(), image.height());

    //Write the magnitude and direction out to images
    writeMagnitude(sobel, magnitude);
    writeMagnitudeDirection(sobel_dir, magnitude, direction);
    writeMagnitude(nms_image, nms);
    writeMagnitudeDirection(nms_image_dir, nms, direction);

    //Write the images out to files
    sobel.save("sobel.bmp");
    sobel_dir.save("sobel_direction.bmp");
    nms_image.save("non_maxima.bmp");
    nms_image_dir.save("non_maxima_dir.bmp");

    //Return successful
    return 0;
}
