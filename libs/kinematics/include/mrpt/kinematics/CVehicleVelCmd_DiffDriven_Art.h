/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/kinematics/CVehicleVelCmd.h>

namespace mrpt
{
namespace kinematics
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CVehicleVelCmd_DiffDriven_Art, CVehicleVelCmd, KINEMATICS_IMPEXP)




    /** Kinematic model for Ackermann-like or differential-driven vehicles with articulation.
     *
     * Geometry of the vehicle is :
     *
     *       8----------------7           2-----------1
     *       |                |           |           |
     *   T_W |                5-----------4     o     | H_W
     *       |                |  tt_leght |           |
     *       9----------------6           3-----------0
     *         trailer_length             tractor_length
     *
     *
     * TODO: it should be possible to bring this model to a kinematic chain to support n-trailers robots
     *
	 * \ingroup mrpt_kinematics_grp
	 */
	class KINEMATICS_IMPEXP CVehicleVelCmd_DiffDriven_Art : public CVehicleVelCmd
	{
		DEFINE_SERIALIZABLE(CVehicleVelCmd_DiffDriven_Art)
	public:
		double lin_vel; //!< Linear velocity (m/s)      //this are already included in CVehicleVelCmd_DiffDriven
		double ang_vel; //!< Angular velocity (rad/s)

        CVehicleVelCmd_DiffDriven_Art();
		virtual ~CVehicleVelCmd_DiffDriven_Art();

		size_t getVelCmdLength() const MRPT_OVERRIDE;
		std::string getVelCmdDescription(const int index) const MRPT_OVERRIDE;
		double getVelCmdElement(const int index) const  MRPT_OVERRIDE;
		void setVelCmdElement(const int index, const double val) MRPT_OVERRIDE;
		bool isStopCmd() const MRPT_OVERRIDE;
		void setToStop() MRPT_OVERRIDE;

		/** See docs of method in base class. The implementation for differential-driven robots of this method
		* just multiplies all the components of vel_cmd times vel_scale, which is appropriate
        * for differential-driven kinematic models (v,w).
		*/
		void cmdVel_scale(double vel_scale) MRPT_OVERRIDE;

		/** See base class docs.
		 * Tecognizes these parameters: `robotMax_V_mps`, `robotMax_W_degps` */
        void cmdVel_limits(const mrpt::kinematics::CVehicleVelCmd &prev_vel_cmd,
                           const double beta,
                           const TVelCmdParams &params)  MRPT_OVERRIDE;

	private:
		void filter_max_vw(double &v, double &w, const TVelCmdParams &p);
	};
    DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(CVehicleVelCmd_DiffDriven_Art, CVehicleVelCmd, KINEMATICS_IMPEXP)

	} // End of namespace
} // End of namespace
