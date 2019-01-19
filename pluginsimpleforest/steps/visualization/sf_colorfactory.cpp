#include "sf_colorfactory.h"

CT_Color SF_ColorFactory::getColor(SF_ColorFactory::Color clr)
{
  switch (clr) {
    case Color::RED:
      return CT_Color( 205, 20, 20, 127);
      break;
    case Color::GREEN:
      return CT_Color{ 20, 205, 20, 127 };
      break;
    case Color::BLUE:
      return CT_Color{ 20, 20, 205, 127 };
      break;
    case Color::YELLOW:
      return CT_Color{ 205, 205, 20, 127 };
      break;
    case Color::VIOLET:
      return CT_Color{ 205, 20, 205, 127 };
      break;
    case Color::CYAN:
      return CT_Color{ 20, 205, 205, 127 };
      break;
    case Color::BRIGHT:
      return CT_Color{ 205, 205, 205, 127 };
      break;
    case Color::DARK:
      return CT_Color{ 20, 20, 20, 127 };
      break;
    default:
      break;
  }
  return CT_Color{ 20, 20, 20, 127 };
}

SF_ColorFactory::SF_ColorFactory() {}
