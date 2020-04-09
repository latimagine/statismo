#include "statismo/VTK/vtkStandardMeshRepresenter.h"
#include "statismo/ITK/itkStandardMeshRepresenter.h"
#include <iostream>

int
main()
{
  std::cout << "TEST: Creating an ITK representer" << std::endl;
  auto itkrep = itk::StandardMeshRepresenter<float, 3>::New();
  std::cout << "TEST: ITK representer created at " << itkrep.GetPointer() << std::endl;

  std::cout << "TEST: Creating a VTK representer" << std::endl;
  auto vtkrep = statismo::vtkStandardMeshRepresenter::SafeCreate();
  std::cout << "TEST: VTK representer created at " << vtkrep.get() << std::endl;

  return EXIT_SUCCESS;
}