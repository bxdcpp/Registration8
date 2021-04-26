#include "VisualITKImageRegistration.h"



//#include <vtkMatrix4x4.h>
#include <itkImageFileReader.h>
#include <itkChangeInformationImageFilter.h>


#include "itkCommand.h"
class CommandIterationUpdate : public itk::Command
{
public:
	using Self = CommandIterationUpdate;
	using Superclass = itk::Command;
	using Pointer = itk::SmartPointer<Self>;
	itkNewMacro(Self);

protected:
	CommandIterationUpdate() = default;

public:
	using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
	using OptimizerPointer = const OptimizerType *;
	void
		Execute(itk::Object * caller, const itk::EventObject & event) override
	{
		Execute((const itk::Object *)caller, event);
	}
	void
		Execute(const itk::Object * object, const itk::EventObject & event) override
	{
		auto optimizer = static_cast<OptimizerPointer>(object);
		if (!itk::IterationEvent().CheckEvent(&event))
		{
			return;
		}
		std::cout << optimizer->GetCurrentIteration() << "   ";
		std::cout << optimizer->GetValue() << "   ";
		std::cout << optimizer->GetCurrentPosition() << std::endl;
	}
};

namespace itk
{

	VisualITKImageRegistration::VisualITKImageRegistration()
		:m_FixedImage(nullptr)
		, m_MovingImage(nullptr)
		,m_Registration(nullptr)
		,m_transform(nullptr)
	{
		m_transform = TransformType::New();
	}

	void VisualITKImageRegistration::GenerateData()
	{
		this->Update();
	}

	void VisualITKImageRegistration::SetupRegistration()
	{
		MetricType::Pointer       metric = MetricType::New();
		metric->SetNumberOfHistogramBins(50);
		metric->SetUseMovingImageGradientFilter(false);
		metric->SetUseFixedImageGradientFilter(false);
		metric->SetUseSampledPointSet(false);


		//MIMetricType::Pointer localCostMetric = dynamic_cast<MIMetricType *>(metric);
		//localCostMetric->SetFixedImage(m_FixedImage);
		//localCostMetric->SetMovingImage(m_MovingImage);
		//localCostMetric->SetVirtualDomainFromImage(m_FixedImage);

		////	using MIMetricType =
		////	itk::MattesMutualInformationImageToImageMetricv4<FixedImageType, MovingImageType, FixedImageType, RealType>;
		////MIMetricType::Pointer mutualInformationMetric = MIMetricType::New();
		////// The next line was a hack for early ITKv4 mattes mutual informaiton
		////// that was using a lot of memory
		////// mutualInformationMetric->SetMaximumNumberOfThreads(std::min(
		////// 3U,itk::MultiThreaderBase::GetGlobalDefaultNumberOfThreads() ) );
		////mutualInformationMetric->SetNumberOfHistogramBins(this->m_NumberOfHistogramBins);
		////mutualInformationMetric->SetUseMovingImageGradientFilter(gradientfilter);
		////mutualInformationMetric->SetUseFixedImageGradientFilter(gradientfilter);
		////mutualInformationMetric->SetUseSampledPointSet(false);
		////metric = mutualInformationMetric;

		/*this->SetupRegistration<MIMetricType>(metric);
		this->RunRegistration<MIMetricType>();
*/





	    m_Optimizer = OptimizerType::New();
		m_Registration = RegistrationType::New();


		m_Registration->SetMetric(metric);
		m_Registration->SetOptimizer(m_Optimizer);


		TransformType::Pointer initialTransform = TransformType::New();
		m_Registration->SetFixedImage(m_FixedImage);
		m_Registration->SetMovingImage(m_MovingImage);

		TransformInitializerType::Pointer initializer = TransformInitializerType::New();
		initializer->SetTransform(initialTransform);
		initializer->SetFixedImage(m_FixedImage);
		initializer->SetMovingImage(m_MovingImage);
		initializer->MomentsOn();
		initializer->InitializeTransform();

		VersorType rotation;
		VectorType axis;
		axis[0] = 0.0;
		axis[1] = 0.0;
		axis[2] = 1.0;
		constexpr double angle = 0;
		rotation.Set(axis, angle);
		initialTransform->SetRotation(rotation);

		m_Registration->SetInitialTransform(initialTransform);

		OptimizerScalesType optimizerScales(initialTransform->GetNumberOfParameters());
		const double        translationScale = 1.0 / 1000.0;
		optimizerScales[0] = 1.0;
		optimizerScales[1] = 1.0;
		optimizerScales[2] = 1.0;
		optimizerScales[3] = translationScale;
		optimizerScales[4] = translationScale;
		optimizerScales[5] = translationScale;
		m_Optimizer->SetScales(optimizerScales);
		m_Optimizer->SetNumberOfIterations(1500);
		m_Optimizer->SetLearningRate(0.2);
		m_Optimizer->SetMinimumStepLength(0.001);
		m_Optimizer->SetReturnBestParametersAndValue(true);

		// Create the Command observer and register it with the m_Optimizer.
		//
		CommandIterationUpdate::Pointer observer = CommandIterationUpdate::New();
		m_Optimizer->AddObserver(itk::IterationEvent(), observer);

		// One level registration process without shrinking and smoothing.

		constexpr unsigned int numberOfLevels = 1;

		RegistrationType::ShrinkFactorsArrayType shrinkFactorsPerLevel;
		shrinkFactorsPerLevel.SetSize(1);
		shrinkFactorsPerLevel[0] = 1;

		RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
		smoothingSigmasPerLevel.SetSize(1);
		smoothingSigmasPerLevel[0] = 0;

		m_Registration->SetNumberOfLevels(numberOfLevels);
		m_Registration->SetSmoothingSigmasPerLevel(smoothingSigmasPerLevel);
		m_Registration->SetShrinkFactorsPerLevel(shrinkFactorsPerLevel);

	}
	int  VisualITKImageRegistration::RunRegistration()
	{
		if (!m_Registration)
			return EXIT_FAILURE;
		try
		{
			m_Registration->Update();
			std::cout << "Optimizer stop condition: "
				<< m_Registration->GetOptimizer()->GetStopConditionDescription()
				<< std::endl;
		}
		catch (const itk::ExceptionObject & err)
		{
			std::cerr << "ExceptionObject caught !" << std::endl;
			std::cerr << err << std::endl;
			return EXIT_FAILURE;
		}
		return EXIT_SUCCESS;
	}
	void VisualITKImageRegistration::GetRegistrationMatrix()
	{
		std::cout << "GetRegistrationMatrix" << std::endl;
		if (m_Registration && m_Optimizer)
		{
			const TransformType::ParametersType finalParameters =
				m_Registration->GetOutput()->Get()->GetParameters();

			const double       versorX = finalParameters[0];
			const double       versorY = finalParameters[1];
			const double       versorZ = finalParameters[2];
			const double       finalTranslationX = finalParameters[3];
			const double       finalTranslationY = finalParameters[4];
			const double       finalTranslationZ = finalParameters[5];
			const unsigned int numberOfIterations = m_Optimizer->GetCurrentIteration();
			const double       bestValue = m_Optimizer->GetValue();

			// Print out results
			//
			std::cout << std::endl << std::endl;
			std::cout << "Result = " << std::endl;
			std::cout << " versor X      = " << versorX << std::endl;
			std::cout << " versor Y      = " << versorY << std::endl;
			std::cout << " versor Z      = " << versorZ << std::endl;
			std::cout << " Translation X = " << finalTranslationX << std::endl;
			std::cout << " Translation Y = " << finalTranslationY << std::endl;
			std::cout << " Translation Z = " << finalTranslationZ << std::endl;
			std::cout << " Iterations    = " << numberOfIterations << std::endl;
			std::cout << " Metric value  = " << bestValue << std::endl;


			m_transform->SetFixedParameters(
				m_Registration->GetOutput()->Get()->GetFixedParameters());
			m_transform->SetParameters(finalParameters);


			// Software Guide : BeginCodeSnippet
			TransformType::MatrixType matrix = m_transform->GetMatrix();
			TransformType::OffsetType offset = m_transform->GetOffset();
			std::cout << "Matrix = " << std::endl << matrix << std::endl;
			std::cout << "Offset = " << std::endl << offset << std::endl;
		}
	}

	//void VisualITKImageRegistration::GetRegistrationMatrix(Matrix4x4Type& M4)
	//{
	//	std::cout << "GetRegistrationMatrix" << std::endl;
	//	if(m_Registration && m_Optimizer)
	//	{
	//		const TransformType::ParametersType finalParameters =
	//			m_Registration->GetOutput()->Get()->GetParameters();

	//		const double       versorX = finalParameters[0];
	//		const double       versorY = finalParameters[1];
	//		const double       versorZ = finalParameters[2];
	//		const double       finalTranslationX = finalParameters[3];
	//		const double       finalTranslationY = finalParameters[4];
	//		const double       finalTranslationZ = finalParameters[5];
	//		const unsigned int numberOfIterations = m_Optimizer->GetCurrentIteration();
	//		const double       bestValue = m_Optimizer->GetValue();

	//	// Print out results
	//	//
	//	std::cout << std::endl << std::endl;
	//	std::cout << "Result = " << std::endl;
	//	std::cout << " versor X      = " << versorX << std::endl;
	//	std::cout << " versor Y      = " << versorY << std::endl;
	//	std::cout << " versor Z      = " << versorZ << std::endl;
	//	std::cout << " Translation X = " << finalTranslationX << std::endl;
	//	std::cout << " Translation Y = " << finalTranslationY << std::endl;
	//	std::cout << " Translation Z = " << finalTranslationZ << std::endl;
	//	std::cout << " Iterations    = " << numberOfIterations << std::endl;
	//	std::cout << " Metric value  = " << bestValue << std::endl;

	//	TransformType::Pointer finalTransform = TransformType::New();

	//	finalTransform->SetFixedParameters(
	//		m_Registration->GetOutput()->Get()->GetFixedParameters());
	//	finalTransform->SetParameters(finalParameters);


	//	// Software Guide : BeginCodeSnippet
	//	TransformType::MatrixType matrix = finalTransform->GetMatrix();
	//	TransformType::OffsetType offset = finalTransform->GetOffset();
	//	std::cout << "Matrix = " << std::endl << matrix << std::endl;
	//	std::cout << "Offset = " << std::endl << offset << std::endl;

	//	M4(0, 0) = matrix(0, 0);
	//	M4(0, 1) = matrix(0, 1);
	//	M4(0, 2) = matrix(0, 2);

	//	M4(1, 0) = matrix(1, 0);
	//	M4(1, 1) = matrix(1, 1);
	//	M4(1, 2) = matrix(1, 2);

	//	M4(2, 0) = matrix(2, 0);
	//	M4(2, 1) = matrix(2, 1);
	//	M4(2, 2) = matrix(2, 2);

	//	M4(0, 3) = offset[0];
	//	M4(1, 3) = offset[1];
	//	M4(2, 3) = offset[2];
	//	M4(3, 0) = 0;
	//	M4(3, 1) = 0;
	//	M4(3, 2) = 0;
	//	M4(3, 3) = 1;
	//	}
	//}

	void VisualITKImageRegistration::Update()
	{
		this->RunRegistration();
	}

}
