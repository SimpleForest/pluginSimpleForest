#include "sf_colorfactory.h"

#include <iostream>

CT_Color
SF_ColorFactory::getColor(SF_ColorFactory::Color color)
{
  switch (color) {
    case Color::RED:
      return CT_Color(205, 20, 20, 127);
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

CT_Color
SF_ColorFactory::getColor(SF_ColorFactory::Color colorOne, SF_ColorFactory::Color colorTwo, float frac)
{
  CT_Color first = getColor(colorOne);
  CT_Color second = getColor(colorTwo);
  if (frac <= 0) {
    return first;
  }
  if (frac >= 1) {
    return second;
  }
  return CT_Color(static_cast<uchar>((double)first.coeff(0) + frac * ((double)second.coeff(0) - (double)first.coeff(0))),
                  static_cast<uchar>((double)first.coeff(1) + frac * ((double)second.coeff(1) - (double)first.coeff(1))),
                  static_cast<uchar>((double)first.coeff(2) + frac * ((double)second.coeff(2) - (double)first.coeff(2))),
                  static_cast<uchar>(127));
}

QString
SF_ColorFactory::getColorString(SF_ColorFactory::Color colorOne, SF_ColorFactory::Color colorTwo, float frac, QString seperator)
{
  CT_Color color = getColor(colorOne, colorTwo, frac);
  QString str;
  str.append(QString::number(static_cast<size_t>(color.coeff(0))));
  str.append(seperator);
  str.append(QString::number(static_cast<size_t>(color.coeff(1))));
  str.append(seperator);
  str.append(QString::number(static_cast<size_t>(color.coeff(2))));
  return str;
}

SF_ColorFactory::SF_ColorFactory() {}
