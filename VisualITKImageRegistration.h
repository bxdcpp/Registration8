#pragma once

// ITK include
#include "itkObject.h"
#include "itkObjectFactory.h"
#include "itkSpatialObject.h"
#include "itkImageRegistrationMethodv4.h"
#include "itkMeanSquaresImageToImageMetricv4.h"
#include "itkMattesMutualInformationImageToImageMetricv4.h"
#include "itkRegularStepGradientDescentOptimizerv4.h"
#include "itkCenteredTransformInitializer.h"
#include "itkVersorRigid3DTransform.h"
#include "itkMatrix.h"
#include <itkImage.h>
#include <itkImageSource.h>
#include <itkVTKImageToImageFilter.h>
#include "itkCastImageFilter.h"
#include "itkAffineTransform.h"

// VTK include
//#include <vtkSmartPointer.h>
//#include <vtkImageData.h>

// QT include
//#include <QColor>

#include <vector>

namespace  itk {
	class  VisualITKImageRegistration : public ProcessObject
	{
	public:
		ITK_DISALLOW_COPY_AND_ASSIGN(VisualITKImageRegistration);

		/** Standard class type alias. */
		using Self = VisualITKImageRegistration;
		using Superclass = ProcessObject;
		using Pointer = SmartPointer<Self>;
		using ConstPointer = SmartPointer<const Self>;


		using RealType = double;
		using PixelType = float;
		using FixedImageType = itk::Image<PixelType, 3>;
		using FixedImageConstPointer = FixedImageType::ConstPointer;
		using FixedImagePointer = FixedImageType::Pointer;

		using MovingImageType = itk::Image<PixelType, 3>;
		using MovingImageConstPointer = MovingImageType::ConstPointer;
		using MovingImagePointer = MovingImageType::Pointer;


		/** Constants for the image dimensions */
		static constexpr unsigned int FixedImageDimension = FixedImageType::ImageDimension;
		static constexpr unsigned int MovingImageDimension = MovingImageType::ImageDimension;


		using FixedBinaryVolumeType = SpatialObject<Self::FixedImageDimension>;
		using MovingBinaryVolumeType = SpatialObject<Self::MovingImageDimension>;
		using FixedBinaryVolumePointer = FixedBinaryVolumeType::Pointer;
		using MovingBinaryVolumePointer = MovingBinaryVolumeType::Pointer;


		using TransformType = itk::VersorRigid3DTransform<double>;

		using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
		//using MetricType =
		//	itk::MeanSquaresImageToImageMetricv4<FixedImageType, MovingImageType>;
		using MetricType =
			itk::MattesMutualInformationImageToImageMetricv4<FixedImageType, MovingImageType, FixedImageType, RealType>;

		using AffineRegistrationType = itk::ImageRegistrationMethodv4<FixedImageType, MovingImageType>;
		using AffineTransformType = itk::AffineTransform<RealType, 3>;
		using SamplingStrategyType = AffineRegistrationType::MetricSamplingStrategyType;

		using RegistrationType =
			itk::ImageRegistrationMethodv4<FixedImageType, MovingImageType>;
		/*using RegistrationType =
			itk::ImageRegistrationMethodv4<FixedImageType, MovingImageType, TransformType>;*/

		using TransformInitializerType =
			itk::CenteredTransformInitializer<TransformType, FixedImageType, MovingImageType>;

		using VersorType = TransformType::VersorType;
		using VectorType = VersorType::VectorType;

		using OptimizerScalesType = OptimizerType::ScalesType;
		using Matrix4x4Type = itk::Matrix<double, 4, 4>;
		//using Matrix4x4Pointer = Matrix4x4Type::Pointer;



		/** Method for creation through the object factory. */
		itkNewMacro(Self);

		/** Run-time type information (and related methods). */
		itkTypeMacro(VisualITKImageRegistration, ProcessObject);


		/** Set/Get the Fixed image. */
		itkSetObjectMacro(FixedImage, FixedImageType);
		itkGetConstObjectMacro(FixedImage, FixedImageType);

		/** Set/Get the Moving image. */
		itkSetObjectMacro(MovingImage, MovingImageType);
		itkGetConstObjectMacro(MovingImage, MovingImageType);

		itkSetMacro(SamplingStrategy, SamplingStrategyType);
		itkGetConstMacro(SamplingStrategy, SamplingStrategyType);

		void SetupRegistration();
		int  RunRegistration();
		//void GetRegistrationMatrix(Matrix4x4Type& M4);
		void GetRegistrationMatrix();

		void Update() override;

	protected:
		VisualITKImageRegistration();
		~VisualITKImageRegistration() override = default;

		/** Method invoked by the pipeline in order to trigger the computation of
		* the registration. */
		void
			GenerateData() override;
	private:

		FixedImagePointer  m_FixedImage;
		FixedImagePointer  m_FixedImage2; // For multi-modal SyN
		MovingImagePointer m_MovingImage;
		MovingImagePointer m_MovingImage2; // For multi-modal SyN
		MovingImagePointer m_PreprocessedMovingImage;
		MovingImagePointer m_PreprocessedMovingImage2; // For multi-modal SyN

		FixedBinaryVolumePointer  m_FixedBinaryVolume;
		FixedBinaryVolumePointer  m_FixedBinaryVolume2; // For multi-modal SyN
		MovingBinaryVolumePointer m_MovingBinaryVolume;
		MovingBinaryVolumePointer m_MovingBinaryVolume2; // For multi-modal SyN
		std::string               m_OutputFixedImageROI;
		std::string               m_OutputMovingImageROI;
		RegistrationType::Pointer m_Registration;
		OptimizerType::Pointer    m_Optimizer;
		TransformType::Pointer    m_transform;

		RealType     m_SamplingPercentage{ 1 };
		unsigned int m_NumberOfHistogramBins{ 50 };
		bool         m_HistogramMatch{ false };
		float        m_RemoveIntensityOutliers{ 0.00 };
		unsigned int m_NumberOfMatchPoints{ 10 };

		// INFO:  Would be better to have unsigned int
		std::vector<int>             m_NumberOfIterations;
		double                       m_MaximumStepLength{ 0.2 };
		std::vector<double>          m_MinimumStepLength;
		double                       m_RelaxationFactor{ 0.5 };
		double                       m_TranslationScale{ 1000.0 };
		double                       m_ReproportionScale{ 1.0 };
		double                       m_SkewScale{ 1.0 };
		double                       m_BackgroundFillValue{ 0.0 };
		std::vector<std::string>     m_TransformType;
		std::string                  m_InitializeTransformMode;
		double                       m_MaskInferiorCutOffFromCenter{ 1000 };
		std::vector<int>             m_SplineGridSize;
		double                       m_CostFunctionConvergenceFactor{ 1e+9 };
		double                       m_ProjectedGradientTolerance{ 1e-5 };
		double                       m_MaxBSplineDisplacement{ 0.0 };
		unsigned int                 m_ActualNumberOfIterations{ 0 };
		unsigned int                 m_PermittedNumberOfIterations{ 0 };
		unsigned int                 m_DebugLevel{ 0 };

		bool                         m_DisplayDeformedImage{ false };
		bool                         m_PromptUserAfterDisplay{ false };
		double                       m_FinalMetricValue{ 0.0 };
		bool                         m_ObserveIterations{ false };
		typename MetricType::Pointer m_CostMetricObject;
//		bool                         m_UseROIBSpline{ false };
		bool                         m_InitializeRegistrationByCurrentGenericTransform{ true };
		int                          m_MaximumNumberOfEvaluations{ 900 };
		int                          m_MaximumNumberOfCorrections{ 12 };
		std::string                  m_SyNMetricType;
		std::string                  m_SaveState;
		bool                         m_SyNFull{ true };
		SamplingStrategyType m_SamplingStrategy;


	};
}


