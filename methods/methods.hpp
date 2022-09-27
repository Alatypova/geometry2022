/**
 * @file methods/methods.hpp
 * @author Mikhail Lozhnikov
 *
 * Объявления функий для серверной части алгоритмов. Эти функции должны
 * считать JSON, который прислал клиент, выполнить алгоритм и отправить клиенту
 * JSON с результатом работы алгоритма.
 */

#ifndef METHODS_METHODS_HPP_
#define METHODS_METHODS_HPP_

namespace geometry {

/* Сюда нужно вставить объявление серверной части алгоритма. */

/**
 * @brief Метод построения контура объединения прямоугольников.
 *
 * @param input Входные данные в формате JSON.
 * @param output Выходные данные в формате JSON.
 * @return Функция возвращает 0 в случае успеха и отрицательное число
 * если входные данные заданы некорректно.
 *
 * Функция запускает алгоритм сортировки вставками, используя входные данные
 * в JSON формате. Результат также выдаётся в JSON формате.
 */
int ContourRectanglesMethod(const nlohmann::json& input,
                            nlohmann::json* output);
int GrahamScanMethod(const nlohmann::json& input,
                            nlohmann::json* output);

/**
 * @brief Метод определения многоугольника Вороного.
 *
 * @param input Входные данные в формате JSON.
 * @param output Выходные данные в формате JSON.
 * @return Функция возвращает 0 в случае успеха и отрицательное число
 * если входные данные заданы некорректно.
 *
 * Функция запускает алгоритм определения многоугольника Вороного,
 * используя входные данные в JSON формате. Результат также выдаётся
 * в JSON формате.
 */
int VoronoyPolygonMethod(const nlohmann::json& input,
                         nlohmann::json* output);


/* Конец вставки. */

}  // namespace geometry

#endif  // METHODS_METHODS_HPP_
