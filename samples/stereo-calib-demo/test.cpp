/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils.h>
#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/system/filesystem.h> // for ASSERT_FILE_EXISTS_
#include <mrpt/vision.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::vision;
using namespace std;


// ------------------------------------------------------
//				TestStereoCalibrate
// ------------------------------------------------------
int TestStereoCalibrate(int argc, char** argv)
{
	CTimeLogger  timlog;

	// Parse optional arguments:
	if (argc==1 || ((argc-1)&1)!=0)
	{
		cout<< "Usage:\n"
			<< argv[0] << "left_image1 right_image1 [L2 R2] [L3 R3] [...]\n";
		return -1;
	}

	// The stereo calibration structures:
	TCalibrationStereoImageList calib_imgs;
	TStereoCalibResults         calib_result;
	TStereoCalibParams          calib_params;

	// Load images:
	const size_t nPairs = (argc>>1);
	for (size_t i=0;i<nPairs;i++)
	{
		const string sImgL = argv[1+2*i+0];
		const string sImgR = argv[1+2*i+1];
		ASSERT_FILE_EXISTS_(sImgL)
		ASSERT_FILE_EXISTS_(sImgR)

		calib_imgs.resize(calib_imgs.size()+1);
		TImageStereoCalibData &stereo_dat = *calib_imgs.rbegin();
		
#if 1
		// Load all images in memory:
		if (!stereo_dat.left.img_original.loadFromFile(sImgL))  THROW_EXCEPTION_CUSTOM_MSG1("Error loading: %s",sImgL.c_str())
		if (!stereo_dat.right.img_original.loadFromFile(sImgR)) THROW_EXCEPTION_CUSTOM_MSG1("Error loading: %s",sImgR.c_str())
#else
		// Don't load images in memory until really needed.
		stereo_dat.left.img_original.setExternalStorage(sImgL);
		stereo_dat.right.img_original.setExternalStorage(sImgR);
#endif
	}

	// Run calibration:
	bool res = mrpt::vision::checkerBoardStereoCalibration( calib_imgs, calib_params, calib_result );

	if (!res) 
	{
		std::cout << "Calibration returned an error status.\n";
		return -1;
	}
	else
	{
		// Calibration was OK:

		// Show detected corners:
		if (1)
		{
			mrpt::gui::CDisplayWindow3D win("Calibration results",1000,480);

			mrpt::opengl::COpenGLViewportPtr view1, view2;
			{
				mrpt::opengl::COpenGLScenePtr &scene = win.get3DSceneAndLock();
				view1 = scene->getViewport("main");
				view2 = scene->createViewport("right");

				// Split viewing area into two halves:
				view1->setViewportPosition(0,0,   0.5,1.0);
				view2->setViewportPosition(0.5,0, 0.5,1.0);

				win.unlockAccess3DScene();
			}

			for (size_t i=0;i<nPairs;i++)
			{
				win.get3DSceneAndLock();

				view1->setImageView( calib_imgs[i].left.img_checkboard );
				view2->setImageView( calib_imgs[i].right.img_checkboard );
				
				win.setWindowTitle(mrpt::format("Detected corners: %u / %u", static_cast<unsigned int>(i+1),static_cast<unsigned int>(nPairs) ));

				win.unlockAccess3DScene();
				win.repaint();

				win.waitForKey();
			}
		} // end show detected corners


		return 0;
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char** argv)
{
	try
	{
		return TestStereoCalibrate(argc,argv);
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}