Формат входных данных
Два числа n, m - количество ыершин и количество ребер
Далее m троек чисел u, v, t - стартовая и конечные вершины и время движения

Единстенное число cnt - количество веток
cnt описаний ветки в формате:
на первой строке два числа: sz и interval - количество вершин и интервал
далее sz чисел - номера вершин маршрута

Единственное число q - количество запросов
Далее идут q строк запросов в формате:
u, v, t - стартовая, конечная и время входа

Пример1:

7 6

1 2 3

2 3 4

3 4 5

4 5 5

5 6 4

6 7 3


1

7 4

1 2 3 4 5 6 7

3

4 1 0

4 6 0

3 5 2

Пример2:

9 8

1 2 3

2 3 3

3 4 3

4 5 3

6 7 3

7 3 3

3 8 3

8 9 3


2

5 3

1 2 3 4 5

5 4

6 7 3 8 9

2

2 9 4

2 9 5

Для сборки запустите build.sh

Для запуска вызовите program
