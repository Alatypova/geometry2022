/**
 * @file tests/voronoy_polygon_test.cpp
 * @author Asel Latypova
 *
 * Реализация набора тестов для алгоритма нахождения многоугольника Вороного.
 */

#include <voronoy_polygon.hpp>
#include <httplib.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include "test_core.hpp"
#include "test.hpp"

static void SimpleTest(httplib::Client* cli);
static void SimpleTest2(httplib::Client* cli);
static void SimpleTest3(httplib::Client* cli);
static void RandomTest(httplib::Client* cli);


template<typename T>
static void RandomFloatingPointHelperTest(httplib::Client* cli,
                                          std::string type);

void TestVoronoyPolygon(httplib::Client* cli) {
  TestSuite suite("TestVoronoyPolygon");

  RUN_TEST_REMOTE(suite, cli, SimpleTest);
  RUN_TEST_REMOTE(suite, cli, SimpleTest2);
  RUN_TEST_REMOTE(suite, cli, SimpleTest3);
  RUN_TEST_REMOTE(suite, cli, RandomTest);
}

/**
 * @brief Простейший статический тест.
 *
 * @param cli Указатель на HTTP клиент.
 */
static void SimpleTest(httplib::Client* cli) {
  {
    /*
    Библиотека nlohmann json позволяет преобразовать
    строку в объект nlohmann::json не только при помощи
    функции nlohmann::json::parse(), но и при помощи
    специального литерала _json. Если его поставить после строки
    в кавычках, то она конвертируется в json объект.

    R"(
    )" Так записывается строка, содержащая символы перевода строки
    в C++. Всё, что между скобками это символы строки. Перводы строк
    можно ставить просто как перевод строки в текстовом редактора
    (а не через \n).
    */
    nlohmann::json input = R"(
    {
    "id": 1,
    "type": "double",
    "point": [0, 0],
    "size": 1,
    "data": [[0, 1]],
    "box": [[0, 0], [0, 1], [1, 1], [1, 0]],
    "precision": 1e-10
    }
    )"_json;

    /* Делаем POST запрос по адресу нашего метода на сервере.
    МBетод dump() используется для преобразования JSON обратно в строку.
    (Можно было сразу строку передать). При передаче JSON данных
    необходимо поставить тип MIME "application/json".
    */

    httplib::Result res = cli->Post("/VoronoyPolygon", input.dump(),
        "application/json");

    /* Используем метод parse() для преобразования строки ответа сервера
    (res->body) в объект JSON. */

    nlohmann::json output = nlohmann::json::parse(res->body);

    /* Проверка результатов алгоритма. */
    REQUIRE_EQUAL(4, output["size"]);
    REQUIRE_EQUAL(1, output["id"]);
    REQUIRE_EQUAL("double", output["type"]);

    REQUIRE_EQUAL(to_string(output["data"][0]), "[0.0,0.5]");
    REQUIRE_EQUAL(to_string(output["data"][1]), "[1.0,0.5]");
    REQUIRE_EQUAL(to_string(output["data"][2]), "[1.0,0.0]");
    REQUIRE_EQUAL(to_string(output["data"][3]), "[0.0,0.0]");
  }
}

static void SimpleTest2(httplib::Client* cli) {
  {
    /*
    Библиотека nlohmann json позволяет преобразовать
    строку в объект nlohmann::json не только при помощи
    функции nlohmann::json::parse(), но и при помощи
    специального литерала _json. Если его поставить после строки
    в кавычках, то она конвертируется в json объект.

    R"(
    )" Так записывается строка, содержащая символы перевода строки
    в C++. Всё, что между скобками это символы строки. Перводы строк
    можно ставить просто как перевод строки в текстовом редактора
    (а не через \n).
    */
    nlohmann::json input = R"(
    {
    "id": 1,
    "type": "double",
    "point": [0, 1],
    "size": 1,
    "data": [[1, 0]],
    "box": [[0, 0], [0, 1], [1, 1], [1, 0]],
    "precision": 1e-10
    }
    )"_json;

    /* Делаем POST запрос по адресу нашего метода на сервере.
    Метод dump() используется для преобразования JSON обратно в строку.
    (Можно было сразу строку передать). При передаче JSON данных
    необходимо поставить тип MIME "application/json".
    */

    httplib::Result res = cli->Post("/VoronoyPolygon", input.dump(),
        "application/json");

    /* Используем метод parse() для преобразования строки ответа сервера
    (res->body) в объект JSON. */

    nlohmann::json output = nlohmann::json::parse(res->body);

    /* Проверка результатов алгоритма. */
    REQUIRE_EQUAL(3, output["size"]);
    REQUIRE_EQUAL(1, output["id"]);
    REQUIRE_EQUAL("double", output["type"]);

    REQUIRE_EQUAL(to_string(output["data"][0]), "[1.0,1.0]");
    REQUIRE_EQUAL(to_string(output["data"][1]), "[0.0,0.0]");
    REQUIRE_EQUAL(to_string(output["data"][2]), "[0.0,1.0]");
  }
}
static void SimpleTest3(httplib::Client* cli) {
  {
    /*
    Библиотека nlohmann json позволяет преобразовать
    строку в объект nlohmann::json не только при помощи
    функции nlohmann::json::parse(), но и при помощи
    специального литерала _json. Если его поставить после строки
    в кавычках, то она конвертируется в json объект.

    R"(
    )" Так записывается строка, содержащая символы перевода строки
    в C++. Всё, что между скобками это символы строки. Перводы строк
    можно ставить просто как перевод строки в текстовом редактора
    (а не через \n).
    */
    nlohmann::json input = R"(
    {
    "id": 1,
    "type": "double",
    "point": [-0.25, 0.5],
    "size": 1,
    "data": [[0.5, 0.75]],
    "box": [[0, 0], [0, 1], [1, 1], [1, 0]],
    "precision": 1e-10
    }
    )"_json;

    /* Делаем POST запрос по адресу нашего метода на сервере.
    Метод dump() используется для преобразования JSON обратно в строку.
    (Можно было сразу строку передать). При передаче JSON данных
    необходимо поставить тип MIME "application/json".
    */

    httplib::Result res = cli->Post("/VoronoyPolygon", input.dump(),
        "application/json");

    /* Используем метод parse() для преобразования строки ответа сервера
    (res->body) в объект JSON. */

    nlohmann::json output = nlohmann::json::parse(res->body);

    /* Проверка результатов алгоритма. */
    REQUIRE_EQUAL(3, output["size"]);
    REQUIRE_EQUAL(1, output["id"]);
    REQUIRE_EQUAL("double", output["type"]);

    REQUIRE_EQUAL(to_string(output["data"][0]), "[0.0,1.0]");
    REQUIRE_EQUAL(to_string(output["data"][1]), "[0.3333333333333333,0.0]");
    REQUIRE_EQUAL(to_string(output["data"][2]), "[0.0,0.0]");
  }
}

/**
 * @brief Простейший случайный тест.
 *
 * @param cli Указатель на HTTP клиент.
 */
static void RandomTest(httplib::Client* cli) {
  RandomFloatingPointHelperTest<double>(cli, "double");
  RandomFloatingPointHelperTest<long double>(cli, "long double");
}

/**
 * @brief Простейший случайный тест для чисел с плавающей точкой.
 *
 * @tparam T Тип данных сортируемых элементов.
 *
 * @param cli Указатель на HTTP клиент.
 * @param type Строковое представление типа данных сортируемых элементов.
 *
 * Функция используется для сокращения кода, необходимого для поддержки
 * различных типов данных.
 */
template<typename T>
static void RandomFloatingPointHelperTest(httplib::Client* cli,
                                          std::string type) {
  // Число попыток.
  const int numTries = 10;
  // Относительная точность сравнения.
  const T eps = 1e-7;
  // Используется для инициализации генератора случайных чисел.
  std::random_device rd;
  // Генератор случайных чисел.
  std::mt19937 gen(rd());
  // Распределение для количества элементов массива.
  std::uniform_int_distribution<size_t> arraySize(1, 20);
  // Распределение для элементов массива.
  std::uniform_real_distribution<T> elem(T(-10'000), T(10'000));
  size_t it = 0;
  for (it = 0; it < numTries; it++) {
    // Получаем случайный размер массива, используя функцию распределения.

    size_t size = arraySize(gen);

    nlohmann::json input;

    input["id"] = it;
    input["type"] = type;
    input["size"] = size;

    std::vector <geometry::Point<T>> data(size);

    for (size_t i = 0; i < size; i++) {
      // Получаем случайный элемент массива, используя функцию распределения.
      data[i].X() = elem(gen);
      data[i].Y() = elem(gen);

      // Записываем элемент в JSON.
      input["data"][i][0] = data[i].X();
      input["data"][i][1] = data[i].Y();
    }
    geometry::Point<T> pointP;
    pointP.X() = elem(gen);
    pointP.Y() = elem(gen);

    input["point"][0] = pointP.X();
    input["point"][1] = pointP.Y();
    input["precision"] = eps;

    input["box"][0][0] = -10000;
    input["box"][0][1] = -10000;
    input["box"][1][0] = -10000;
    input["box"][1][1] = 10000;
    input["box"][2][0] = 10000;
    input["box"][2][1] = 10000;
    input["box"][3][0] = 10000;
    input["box"][3][1] = -10000;
    /* Отправляем данные на сервер POST запросом. */
    httplib::Result res = cli->Post("/VoronoyPolygon", input.dump(),
        "application/json");

    /* Используем метод parse() для преобразования строки ответа сервера
    (res->body) в объект JSON. */
    nlohmann::json output = nlohmann::json::parse(res->body);

    /* Проверка результатов сортировки. */

    geometry::Point<T> pointQ;
    geometry::Point<T> point;
    bool isVoronoy = 1;
    for (size_t i = 0; i < output["size"]; i++) {
      point.X() = output["data"][i][0];
      point.Y() = output["data"][i][1];
      for (size_t j = 0; j < size; j++) {
        pointQ = data[j];
        if (midPerp(pointP, pointQ, point) > eps) {
          isVoronoy = 0;
          break;
        }
      }
      if (isVoronoy == 0) break;
    }
    REQUIRE_EQUAL(isVoronoy, 1);
  }
}
