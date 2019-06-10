#ifndef SF_COLORFACTORY_H
#define SF_COLORFACTORY_H

#include <ct_color.h>

class SF_ColorFactory
{
public:
  enum class Color
  {
    RED,
    GREEN,
    BLUE,
    YELLOW,
    VIOLET,
    CYAN,
    BRIGHT,
    DARK
  };
  static CT_Color getColor(Color color);
  static CT_Color getColor(Color colorOne, Color colorTwo, float frac);
  static QString getColorString(Color colorOne, Color colorTwo, float frac, QString seperator = QString(" "));

private:
  SF_ColorFactory();
};

#endif // SF_COLORFACTORY_H
