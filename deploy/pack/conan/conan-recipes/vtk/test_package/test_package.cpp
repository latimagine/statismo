#include <vtkConeSource.h>
#include <vtkSmartPointer.h>

#include <iostream>

int
main(int, char * [])
{
  std::cout << "TEST: Creating a cone source" << std::endl;
  vtkSmartPointer<vtkConeSource> coneSource = vtkSmartPointer<vtkConeSource>::New();
  coneSource->Update();

  std::cout << "TEST: Cone source created at " << coneSource.GetPointer() << std::endl;

  return EXIT_SUCCESS;
}
