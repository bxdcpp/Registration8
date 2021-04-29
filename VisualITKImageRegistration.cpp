#include "VisualITKImageRegistration.h"
#include "itkImageRegistrationMethodv4.h"



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
		//指定用于计算熵的箱数，一般设置50，这个值能对优化器产生显著影响
		metric->SetNumberOfHistogramBins(50);
		//为了计算图像梯度，使用了一个基于图像函数的图像梯度计算器来代替图像梯度滤波器。
		//图像梯度方法在超类ImageToImageMetricv4中定义
		//所以设置为false
		metric->SetUseMovingImageGradientFilter(false);
		metric->SetUseFixedImageGradientFilter(false);
		//禁用采样，使用Fixed空间的所有像素
		metric->SetUseSampledPointSet(false);

		metric->SetFixedImage(m_FixedImage);
		metric->SetMovingImage(m_MovingImage);
		metric->SetVirtualDomainFromImage(m_FixedImage);



	    m_Optimizer = OptimizerType::New();
		m_Registration = RegistrationType::New();


		m_Registration->SetMetric(metric);
		m_Registration->SetOptimizer(m_Optimizer);



		m_Registration->SetFixedImage(m_FixedImage);
		m_Registration->SetMovingImage(m_MovingImage);

		TransformType::Pointer initialTransform = TransformType::New();


		TransformInitializerType::Pointer initializer = TransformInitializerType::New();

		using FixedImageCalculatorType = typename itk::ImageMomentsCalculator<FixedImageType>;
		typename FixedImageCalculatorType::Pointer fixedCalculator = FixedImageCalculatorType::New();
		fixedCalculator->SetImage(m_FixedImage);
		fixedCalculator->Compute();
		typename FixedImageCalculatorType::VectorType fixedCenter = fixedCalculator->GetCenterOfGravity();

		const unsigned int                     numberOfFixedParameters = initialTransform->GetFixedParameters().Size(); // =3
		typename TransformType::ParametersType fixedParameters(numberOfFixedParameters);
		for (unsigned int i = 0; i < numberOfFixedParameters; ++i)
		{
			fixedParameters[i] = fixedCenter[i];
		}

	/*	using MatrixType = itk::Matrix<double, 3 + 1, 3 + 1>;
		MatrixType matrix;
		matrix[0][0] = 0.993739;
		matrix[0][1] = -0.110005;
		matrix[0][2] = -0.0195393;

		matrix[1][0] = 0.111443;
		matrix[1][1] = 0.988404;
		matrix[1][2] = 0.103137;

		matrix[2][0] = 0.00796706;
		matrix[2][1] = -0.104669;
		matrix[2][2] = 0.994475;

		matrix[0][3] = 18;
		matrix[1][3] = -8.0;
		matrix[2][3] = -8.0;*/

		/*	Matrix =
				0.993739 - 0.110005 - 0.0195393
				0.111443 0.988404 0.103137
				0.00796706 - 0.104669 0.994475

				Offset =
				[20.5706, -8.92841, -8.23361]*/



		// get transform parameters from MatrixType
		/*TransformType::ParametersType parameters(3 * 3 + 3);
		for (unsigned int i = 0; i < 3; i++)
		{
			for (unsigned int j = 0; j < 3; j++)
			{
				parameters[i * 3 + j] = matrix[i][j];
			}
		}
		for (unsigned int i = 0; i < 3; i++)
		{
			parameters[i + 3 * 3] = matrix[i][3];
		}*/
		//initialTransform->SetParameters(parameters);

		initialTransform->SetFixedParameters(fixedParameters);

		std::cout <<"Parameters:"<< initialTransform->GetParameters().size() << std::endl;

	/*	initializer->SetTransform(initialTransform);
		initializer->SetFixedImage(m_FixedImage);
		initializer->SetMovingImage(m_MovingImage);
		initializer->MomentsOn();
		initializer->InitializeTransform();*/

	




		/*	VersorType rotation;
			VectorType axis;
			axis[0] = 0.0;
			axis[1] = 0.0;
			axis[2] = 1.0;
			constexpr double angle = 0;
			rotation.Set(axis, angle);
			initialTransform->SetRotation(rotation);*/

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
		//m_Optimizer->SetLearningRate(0.2);
		//最小步长
		m_Optimizer->SetMinimumStepLength(0.001);
		m_Optimizer->SetReturnBestParametersAndValue(true);

		//学习速率，大值会使优化器不稳定，太小迭代次数太多,很可能在{1.0,5.0}的范围内
		//一旦调整了其他注册参数以产生收敛性，您可能需要重新访问学习速率，并开始增加其值，
		//直到您观察到优化变得不稳定。该参数的理想值是导致最小的迭代次数，
		//同时仍然在优化的参数空间上保持稳定的路径。
		//请记住，这个参数是一个应用于度规梯度的乘法因子。
		//因此，它对优化器步长的影响与度量值本身成正比。具有大值的度量将需要您对学习速率使用较小的值，以保持类似的优化器行为。
		m_Optimizer->SetLearningRate(0.05);
		//每当规则的步长梯度下降优化器在参数空间中遇到运动方向的变化时，
		//它就会减小步长的大小。步长的速率由松弛因子控制。该系数的默认值为0.5
		m_Optimizer->SetRelaxationFactor(0.5);

		m_Optimizer->SetGradientMagnitudeTolerance(1e-4);
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
		//设置收缩因子
		m_Registration->SetShrinkFactorsPerLevel(shrinkFactorsPerLevel);
		m_Registration->SetSmoothingSigmasAreSpecifiedInPhysicalUnits(true);
		/*shrinkFactorsPerDimensionForAllLevels[level] = {1,1,1}*/
		using ShrinkFactorsPerDimensionContainerType = typename RegistrationType::ShrinkFactorsPerDimensionContainerType;
		ShrinkFactorsPerDimensionContainerType shrinkFactorsPerDimension(3);
		shrinkFactorsPerDimension.Fill(1);
		//为每层设置收缩系数
		m_Registration->SetShrinkFactorsPerDimension(0, shrinkFactorsPerDimension);

		/*std::string metricSamplingStrategy = "Random";
		this->SetSamplingStrategy(metricSamplingStrategy);*/
		//抽样策略 
		m_Registration->SetMetricSamplingStrategy(
			typename RegistrationType::MetricSamplingStrategyType::RANDOM);
		//抽样百分比
		m_Registration->SetMetricSamplingPercentage(0.002);
		//空间点集是伪随机生成的。对于可重复的结果，应该设置一个整数种子。
		m_Registration->MetricSamplingReinitializeSeed(121212);




//		m_Registration->SetReproportionScale(this->m_ReproportionScale)
		//m_Registration->SetSamplingStrategy(m_SamplingStrategy);
		//m_Registration->SetSamplingStrategy("RANDOM");
//		m_Registration->SetRelaxationFactor(m_RelaxationFactor);
//		m_Registration->SetMaximumStepLength(m_MaximumStepLength);
		//m_Registration->SetSkewScale(m_SkewScale);
//		m_Registration->SetBackgroundFillValue(m_BackgroundFillValue);
//		m_Registration->SetDisplayDeformedImage(m_DisplayDeformedImage);
		//m_Registration->Initialize();


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

			const TransformType::ParametersType fiexdParameters = m_Registration->GetOutput()->Get()->GetFixedParameters();
			size_t n = fiexdParameters.size();
			for (int i= 0; i < n; i++)
			{
				std::cout << "fixedParameters:" << "[" << i << "," << fiexdParameters[i] << "]" << std::endl;
			}
		

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
