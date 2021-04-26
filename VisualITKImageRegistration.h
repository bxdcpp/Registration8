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



		using RegistrationType =
			itk::ImageRegistrationMethodv4<FixedImageType, MovingImageType, TransformType>;

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

	};
}


