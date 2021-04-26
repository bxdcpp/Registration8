/*=========================================================================
 *
 *  Copyright NumFOCUS
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0.txt
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *=========================================================================*/

 //  Software Guide : BeginCommandLineArgs
 //    INPUTS: brainweb1e1a10f20.mha
 //    INPUTS: brainweb1e1a10f20Rot10Tx15.mha
 //    ARGUMENTS: ImageRegistration8Output.mhd
 //    ARGUMENTS: ImageRegistration8DifferenceBefore.mhd
 //    ARGUMENTS: ImageRegistration8DifferenceAfter.mhd
 //    OUTPUTS: {ImageRegistration8Output.png}
 //    OUTPUTS: {ImageRegistration8DifferenceBefore.png}
 //    OUTPUTS: {ImageRegistration8DifferenceAfter.png}
 //    OUTPUTS: {ImageRegistration8RegisteredSlice.png}
 //  Software Guide : EndCommandLineArgs

 // Software Guide : BeginLatex
 //
 // This example illustrates the use of the \doxygen{VersorRigid3DTransform}
 // class for performing registration of two $3D$ images.
 // The class \doxygen{CenteredTransformInitializer} is used to initialize the
 // center and translation of the transform.  The case of rigid registration of
 // 3D images is probably one of the most common uses of image registration.
 //
 // \index{itk::Versor\-Rigid3D\-Transform}
 // \index{itk::Centered\-Transform\-Initializer!In 3D}
 //
 // Software Guide : EndLatex
 //ITK
#include "itkMultiThreader.h"
#include "itkImageRegistrationMethodv4.h"
#include "itkMeanSquaresImageToImageMetricv4.h"
#include "VisualITKImageRegistration.h"

//  Software Guide : BeginLatex
//
//  The following are the most relevant headers of this example.
//
//  \index{itk::Versor\-Rigid3D\-Transform!header}
//  \index{itk::Centered\-Transform\-Initializer!header}
//
//  Software Guide : EndLatex

// Software Guide : BeginCodeSnippet
#include "itkVersorRigid3DTransform.h"
#include "itkCenteredTransformInitializer.h"
// Software Guide : EndCodeSnippet

//  Software Guide : BeginLatex
//
//  The parameter space of the \code{VersorRigid3DTransform} is not a vector
//  space, because addition is not a closed operation in the space
//  of versor components. Hence, we need to use Versor composition operation to
//  update the first three components of the parameter array (rotation parameters),
//  and Vector addition for updating the last three components of the parameters
//  array (translation parameters)~\cite{Hamilton1866,Joly1905}.
//
//  In the previous version of ITK, a special optimizer,
//  \doxygen{VersorRigid3DTransformOptimizer} was needed for registration to deal with
//  versor computations. Fortunately in ITKv4, the
//  \doxygen{RegularStepGradientDescentOptimizerv4} can be used for both vector and
//  versor transform optimizations because, in the new registration framework, the task
//  of updating parameters is delegated to the moving transform itself. The
//  \code{UpdateTransformParameters} method is implemented in the \doxygen{Transform}
//  class as a virtual function, and all the derived transform classes can have their
//  own implementations of this function. Due to this fact, the updating function is
//  re-implemented for versor transforms so it can handle versor composition of the
//  rotation parameters.
//
//  Software Guide : EndLatex

// Software Guide : BeginCodeSnippet
#include "itkRegularStepGradientDescentOptimizerv4.h"
// Software Guide : EndCodeSnippet

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkResampleImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkSubtractImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkMatrix.h"


//#include "BRAINSThreadControl.h"

//  The following section of code implements a Command observer
//  that will monitor the evolution of the registration process.
//

int
main(int argc, char * argv[])
{

	itk::MultiThreader::SetGlobalDefaultNumberOfThreads(6);
	if (argc < 1)
	{
		std::cerr << "Missing Parameters " << std::endl;
		std::cerr << "Usage: " << argv[0];
		std::cerr << " fixedImageFile  movingImageFile ";
		std::cerr << " outputImagefile  [differenceBeforeRegistration] ";
		std::cerr << " [differenceAfterRegistration] ";
		std::cerr << " [sliceBeforeRegistration] ";
		std::cerr << " [sliceDifferenceBeforeRegistration] ";
		std::cerr << " [sliceDifferenceAfterRegistration] ";
		std::cerr << " [sliceAfterRegistration] " << std::endl;
		return EXIT_FAILURE;
	}
	constexpr unsigned int Dimension = 3;
	using PixelType = float;
	using FixedImageType = itk::Image<PixelType, Dimension>;
	using MovingImageType = itk::Image<PixelType, Dimension>;

	using ImageRegistrationType = itk::VisualITKImageRegistration;
	ImageRegistrationType::Pointer registration = ImageRegistrationType::New();

	using FixedImageReaderType = itk::ImageFileReader<FixedImageType>;
	using MovingImageReaderType = itk::ImageFileReader<MovingImageType>;
	FixedImageReaderType::Pointer  fixedImageReader = FixedImageReaderType::New();
	MovingImageReaderType::Pointer movingImageReader = MovingImageReaderType::New();

	fixedImageReader->SetFileName(argv[1]);
	movingImageReader->SetFileName(argv[2]);

	registration->SetFixedImage(fixedImageReader->GetOutput());
	registration->SetMovingImage(movingImageReader->GetOutput());

	registration->SetupRegistration();
	try
	{
		registration->Update();
	}
	catch (const itk::ExceptionObject & e)
	{
		std::cerr << "Error: " << e << std::endl;
	}
	registration->GetRegistrationMatrix();

	return EXIT_SUCCESS;
}
