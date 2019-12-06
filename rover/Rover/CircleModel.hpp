#pragma once

#include "AbstractModel.hpp"

typedef std::array<GRANSAC::VPFloat, 2> Vector2VP;

class Point2D
	: public GRANSAC::AbstractParameter
{
public:
	Point2D(GRANSAC::VPFloat x, GRANSAC::VPFloat y)
	{
		m_Point2D[0] = x;
		m_Point2D[1] = y;
	};

	Vector2VP m_Point2D;
};

class Circle2DModel
	: public GRANSAC::AbstractModel<3>
{
public:
	GRANSAC::VPFloat m_x;
	GRANSAC::VPFloat m_y;
	GRANSAC::VPFloat m_r;
protected:

	// Parametrization x^2 + y^2 = r^2
	


	virtual GRANSAC::VPFloat ComputeDistanceMeasure(std::shared_ptr<GRANSAC::AbstractParameter> Param) override
	{
		auto ExtPoint2D = std::dynamic_pointer_cast<Point2D>(Param);
		if (ExtPoint2D == nullptr)
			throw std::runtime_error("Line2DModel::ComputeDistanceMeasure() - Passed parameter are not of type Point2D.");

		// https://www.varsitytutors.com/hotmath/hotmath_help/topics/shortest-distance-between-a-point-and-a-circle
		GRANSAC::VPFloat Dist = abs(sqrt(pow(ExtPoint2D->m_Point2D[0] - m_x, 2) + pow(ExtPoint2D->m_Point2D[1] - m_y, 2)) - m_r);

		//// Debug
		//std::cout << "Point: " << ExtPoint2D->m_Point2D[0] << ", " << ExtPoint2D->m_Point2D[1] << std::endl;
		//std::cout << "Circle: " << m_x << " x + " << m_y << " y + "  << m_r << std::endl;
		//std::cout << "Distance: " << Dist << std::endl << std::endl;

		return Dist;
	};

public:
	Circle2DModel(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> &InputParams)
	{
		Initialize(InputParams);
	};

	virtual void Initialize(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> &InputParams) override
	{
		if (InputParams.size() != 3)
			throw std::runtime_error("Circle2DModel - Number of input parameters does not match minimum number required for this model.");

		// Check for AbstractParamter types
		auto Point1 = std::dynamic_pointer_cast<Point2D>(InputParams[0]);
		auto Point2 = std::dynamic_pointer_cast<Point2D>(InputParams[1]);
		auto Point3 = std::dynamic_pointer_cast<Point2D>(InputParams[2]);

		if (Point1 == nullptr || Point2 == nullptr || Point3 == nullptr)
			throw std::runtime_error("Circle2DModel - InputParams type mismatch. It is not a Point2D.");

		std::copy(InputParams.begin(), InputParams.end(), m_MinModelParams.begin());

		// Compute the circle parameters

		float x_1 = Point1->m_Point2D[0];
		float y_1 = Point1->m_Point2D[1];

		float x_2 = Point2->m_Point2D[0];
		float y_2 = Point2->m_Point2D[1];

		float x_3 = Point3->m_Point2D[0];
		float y_3 = Point3->m_Point2D[1];

		float m_a = (y_2 - y_1) / (x_2 - x_1);
		float m_b = (y_3 - y_2) / (x_3 - x_2);
		
		m_x = (((m_a * m_b) * (y_1 - y_3)) + (m_b * (x_1 + x_2)) - (m_a * (x_2 + x_3))) /
			(2 * (m_b - m_a));
		m_y = (-(1 / m_a) * (m_x - (x_1 + x_2) / 2) + (y_1 + y_2) / 2);

		m_r = sqrt(pow((m_x - x_1),2) + pow((m_y - y_1),2));

	//m_DistDenominator = sqrt(m_a * m_a + m_b * m_b); // Cache square root for efficiency
	};

	virtual std::pair<GRANSAC::VPFloat, std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>> Evaluate(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>& EvaluateParams, GRANSAC::VPFloat Threshold, GRANSAC::VPFloat min_radius, GRANSAC::VPFloat max_radius)
	{
		std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> Inliers;
		int nTotalParams = EvaluateParams.size();
		int nInliers = 0;

		for (auto& Param : EvaluateParams)
		{
			if (ComputeDistanceMeasure(Param) < Threshold)
			{
				Inliers.push_back(Param);
				nInliers++;
			}
		}
		float pi = 3.14159;
		//GRANSAC::VPFloat ideal_circle_total = (pi * pow((m_r + Threshold), 2)) - (pi * pow((m_r - Threshold), 2));
		GRANSAC::VPFloat InlierFraction = GRANSAC::VPFloat(nInliers) / GRANSAC::VPFloat(nTotalParams); // This is the inlier fraction

		if (m_r > max_radius || m_r < min_radius) { return std::make_pair(-1, Inliers); }
		return std::make_pair(InlierFraction, Inliers);
	};
};

