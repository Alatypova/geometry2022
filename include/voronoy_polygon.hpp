/**
 * @file include/voronoy_polygon.hpp
 * @author Asel Latypova
 *
 * Реализация алгоритма нахождения многоугольника Вороного.
 */

#ifndef INCLUDE_VORONOY_POLYGON_HPP_
#define INCLUDE_VORONOY_POLYGON_HPP_

#include <cstddef>
#include <string>
#include <iostream>
#include <list>
#include "point.hpp"
#include "polygon.hpp"

namespace geometry {

/**
 * @brief Функция, возвращающая значение уравнения серединного
 * перпендикуляра отрезка между двумя точками p и q в точке s.
 *
 * @tparam T Тип данных координат.
 *
 * @param p Точка - один из концов отрезка.
 * @param q Точка - другой из концов отрезка.
 * @param s Точка, в которой нужно посчиать значение уравнения.
 */
template<typename T>
T midPerp(const Point<T> &p, const Point<T> &q, const Point<T> &s) {
  return (q.X() - p.X())*(2*s.X() - p.X() - q.X()) +
  (q.Y() - p.Y())*(2*s.Y() - p.Y() - q.Y());
}

/**
 * @brief Функция, которая определяет, нужно ли добавлять точку.
 *
 * @tparam T Тип данных координат.
 *
 * @param edge Ребро.
 * @param p Точка - один из концов отрезка.
 * @param q Точка - другой из концов отрезка.
 * @param eps Точность.
 *
 * Точка, которую, возможно, придется добавить - точка
 * пересечения ребра и серединного перпендикуляра к отрезку.
 */

template<typename T>
bool weNeedToAddThePoint(Edge<T> edge,
                         const Point<T> &p,
                         const Point<T> &q,
                         long double eps) {
  T x1 = edge.Origin().X();
  T y1 = edge.Origin().Y();
  T x2 = edge.Destination().X();
  T y2 = edge.Destination().Y();
  T a = 2*(q.X() - p.X());
  T b = 2*(q.Y() - p.Y());
  T c = y2 - y1;
  T d = -(x2 - x1);
  if (std::abs(a*d - b*c) > eps) {
    return 1;
  }
  return 0;
}

/**
 * @brief Функция, возвращающая точку пересечения двух прямых,
 * одна из которых - серединный перпендикуляр отрезка между
 * точками p и q, а другая - прямая, проходящая через данное ребро.
 *
 * @tparam T Тип данных координат.
 *
 * @param p Точка - один из концов отрезка.
 * @param q Точка - другой из концов отрезка.
 * @param edge Ребро, через которое проходит одна из прямых.
 * @param eps Точность вычислений.
 */

template<typename T>
Point<T> IntersectionPoint(Edge<T> edge,
                           const Point<T> &p,
                           const Point<T> &q,
                           long double eps) {
  Point<T> S;
  T x1 = edge.Origin().X();
  T y1 = edge.Origin().Y();
  T x2 = edge.Destination().X();
  T y2 = edge.Destination().Y();
  T a = 2*(q.X() - p.X());
  T b = 2*(q.Y() - p.Y());
  T c = y2 - y1;
  T d = -(x2 - x1);
  S.X() = ((q.X()*q.X() - p.X()*p.X() +
  q.Y()*q.Y() - p.Y()*p.Y())*d - (x1*y2 - y1*x2)*b)/(a*d - b*c);
  S.Y() = (a*(x1*y2 - y1*x2) - (q.X()*q.X()
  - p.X()*p.X() + q.Y()*q.Y() - p.Y()*p.Y())*c)/(a*d - b*c);
  return S;
}

/**
 * @brief Функция, которая находит пересечение заданного
 * многоугольника и нужной полуплоскости, нужная полуплоскость
 * - та полуплоскость, на которые серединный перпендикуляр отрезка,
 * соединяющего точки p и q, делит плоскость, которая содержит p.
 *
 * @tparam T Тип данных координат.
 *
 * @param p Фиксированная точка.
 * @param q Другая точка.
 * @param *polygon Заданный многоугольник.
 * @param eps Точность вычислений.
 */

template<typename T>
void halfplaneIntersect(const Point<T> &q,
                        Polygon<T, std::list<Point<T>>> *polygon,
                        const Point<T> &p,
                        long double eps) {
  int typeOfFirstPoint = 0;
  int typeOfLastPoint = 0;
  auto first = polygon->Current();
  auto last = polygon->Current();
  auto firstPoint = polygon->Current();
  auto secondPoint = polygon->ClockWise();
  auto zeroPoint = polygon->CounterClockWise();
  auto fixpoint = polygon->Current();
  do {
    if ((midPerp(p, q, *firstPoint) < -eps)
    && (midPerp(p, q, *zeroPoint) > eps)) {
      first = polygon->Current();
      typeOfFirstPoint = 1;
    }
    if ((midPerp(p, q, *firstPoint) < -eps)
    && (midPerp(p, q, *secondPoint) > eps)) {
      last = polygon->Current();
      typeOfLastPoint = 1;
    }
    if ((std::abs(midPerp(p, q, *firstPoint)) < eps)
    && (midPerp(p, q, *zeroPoint) > eps)) {
      first = polygon->Current();
      typeOfFirstPoint = 2;
    }
    if ((std::abs(midPerp(p, q, *firstPoint)) < eps) &&
       (midPerp(p, q, *secondPoint) > eps)) {
      last = polygon->Current();
      typeOfLastPoint = 2;
    }
    if ((typeOfFirstPoint != 0) && (typeOfLastPoint != 0)) break;
    polygon->Current() = polygon->ClockWise();
    firstPoint = polygon->Current();
    secondPoint = polygon->ClockWise();
    zeroPoint = polygon->CounterClockWise();
  } while (polygon->Current() != fixpoint);
  if ((typeOfFirstPoint == 0)
  && (midPerp(p, q, *(polygon->Current())) < eps)) return;
  if (typeOfFirstPoint == 0) {
    Polygon<T, std::list<Point<T>>> polygonForHelp;
    *polygon = polygonForHelp;
    return;
  }
  polygon->Current() = first;
  if ((typeOfFirstPoint == 2) && (typeOfLastPoint == 2) && (first == last)) {
    Polygon<T, std::list<Point<T>>> polygonForHelp;
    *polygon = polygonForHelp;
    return;
  }
  if ((typeOfFirstPoint == 2) && (typeOfLastPoint == 2)
  && (polygon->ClockWise() == last)) {
    Polygon<T, std::list<Point<T>>> polygonForHelp;
    *polygon = polygonForHelp;
    return;
  }
  Polygon<T, std::list<Point<T>>> polygonHelper;
  Point<T> point;
  polygon->Current() = polygon->CounterClockWise();
  Edge<T> firstEdge = polygon->GetEdge();
  polygon->Current() = last;
  Edge<T> lastEdge = polygon->GetEdge();
  polygonHelper = polygon->Split(first);
  if ((typeOfFirstPoint == 1) && (typeOfLastPoint == 1) && (first == last)) {
    *polygon = polygonHelper;
  }
  bool k = 0;
  if ((typeOfFirstPoint == 1) && (weNeedToAddThePoint(firstEdge, p, q, eps))) {
      point = IntersectionPoint(firstEdge, p, q, eps);
      auto it1 = polygon->Insert(point);
      polygon->Current() = polygon->CounterClockWise();
  }
  if ((typeOfLastPoint == 1) && (weNeedToAddThePoint(lastEdge, p, q, eps))) {
    point = IntersectionPoint(lastEdge, p, q, eps);
    auto it2 = polygon->Insert(point);
  }
}

/**
 * @brief Функция, находящая многоугольник Вороного
 * путем пересекания полуплоскостей.
 *
 * @tparam T Тип данных координат.
 *
 * @param p Фиксированная точка.
 * @param s Множество точек.
 * @param n Количество точек в множестве s.
 * @param &box Ограничивающий прямоугольник.
 * @param eps Точность вычислений.
 */

template<typename T>
Polygon<T, std::list<Point<T>>> *voronoyRegion(const Point<T> &p,
                            Point<T>* s, size_t n,
                            const Polygon<T, std::list<Point<T>>>
                            &box, long double eps) {
  auto *r = new Polygon <T, std::list<Point <T>>>(box);
  for (size_t i = 0; i < n; i++) {
    if (p.IsEqual(p, s[i], eps)) {
      auto *polygonForHelp = new Polygon <T, std::list<Point<T>>>();
      delete r;
      return polygonForHelp;
    }
    halfplaneIntersect(s[i], r, p, eps);
    if (r->Size() == 0) {
      break;
    }
  }
  return r;
}

}  // namespace geometry
#endif  // INCLUDE_VORONOY_POLYGON_HPP_
