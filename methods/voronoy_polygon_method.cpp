/*** @file methods/voronoy_polygon_method.cpp
 * @author Asel Latypova
 *
 * Файл содержит функцию, которая вызывает алгоритм нахождения многоугольника Вороного.
 * Функция принимает и возвращает данные в JSON формате.
 */

#include <string>
#include <nlohmann/json.hpp>
#include "voronoy_polygon.hpp"

namespace geometry {

template<typename T>
static int VoronoyPolygonMethodHelper(const nlohmann::json& input,
                                     nlohmann::json* output,
                                     std::string type, long double eps);

int VoronoyPolygonMethod(const nlohmann::json& input, nlohmann::json* output) {
  /*
  С классом nlohmann::json можно работать как со словарём.
B  Метод at() в отличие оператора [] не меняет объект, поэтому
  этот метод можно использовать с константными объектами.
  */
  std::string type = input.at("type");
  long double eps = input.at("precision");

  /* Пока реализована только поддержка чисел типа double и long double. */
  if (type == "double"){
    return VoronoyPolygonMethodHelper<double>(input, output, type, eps);
  } else if (type == "long double") {
    return VoronoyPolygonMethodHelper<long double>(input, output, type, eps);
  }

  return -1;
}

/**
 * @brief Метод определения многоугольника Вороного.
 *
 * @tparam T Тип данных координат вершин. 
 *
 * @param input Входные данные в формате JSON.
 * @param output Выходные данные в формате JSON.
 * @param type Строковое представление типа данных координат.
 * @return Функция возвращает 0 в случае успеха и отрицательное число
 * если входные данные заданы некорректно.
 *
 * Функция запускает алгоритм нахождения многоугольника Вороного,
 * используя входные данные
 * в JSON формате. Результат также выдаётся в JSON формате. Функция
 * используется для сокращения кода, необходимого для
 * поддержки различных типов данных.
 */
template<typename T>
static int VoronoyPolygonMethodHelper(const nlohmann::json& input,
                                     nlohmann::json* output,
                                     std::string type, long double eps) {
  (*output)["id"] = input.at("id");

  size_t size = input.at("size");

  std::list<Point<T>> vertices;

  T x, y;

  for (size_t i = 0; i < 4; i++) {
    x = input.at("box").at(i).at(0);
    y = input.at("box").at(i).at(1);
    Point<T> p1(x, y);
    vertices.push_back(p1);
  }

  Polygon<T, std::list<Point<T>>> box(vertices);

  x = input.at("point").at(0);
  y = input.at("point").at(1);
  Point<T> p(x, y);

  Point<T>* data = new Point<T>[size];

  for (size_t i = 0; i < size; i++) {
    /* Для словарей используется индекс в виде строки,
    а для массивов просто целое число типа size_t. */
    x = input.at("data").at(i).at(0);
    y = input.at("data").at(i).at(1);
    Point<T> k(x, y);
    data[i] = k;
  }
  /*B Здесь вызывается сам алгоритм. */

  Polygon<T, std::list<Point<T>>>* r = voronoyRegion(p, data, size, box, eps);

 /* Сохраняем в ответе результат работы алгоритма. */
  size_t sizeOfR = r->Size();

  (*output)["size"] = sizeOfR;
  (*output)["type"] = type;

  for (size_t i = 0; i < sizeOfR; i++) {
    (*output)["data"][i][0] = (*(r->Current())).X();
    (*output)["data"][i][1] = (*(r->Current())).Y();
    r->Current() = r->ClockWise();
  }

  delete[] data;
  delete r;

  return 0;
}

}  // namespace geometry
