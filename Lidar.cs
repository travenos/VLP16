using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;

namespace HelloWorldOpenCV
{
    /// <summary>
    /// Класс для обработки данных с лидара VLP-16
    /// </summary>
    class Lidar
    {
        private byte[] lidarData = null;        //Необработанный пакет с лидара
        private byte[] prevLidarData = null;    //Новый пакет с лидара

        /// <summary>
        /// Порт подключения к лидару
        /// </summary>
        public ushort Port =2368;

        /// <summary>
        /// Значения вертикальных углов для различных каналов
        /// </summary>
        private static readonly double[] VerticalAngles = {-15, 1, -13, -3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15};

        /// <summary>
        /// Конструктор по умолчанию
        /// </summary>
        /// <param name="hostname">IP-адрес лидара</param>
        /// <param name="port">Порт, через который передаются данные с лидара</param>
        public Lidar(ushort port = 2368)
        {
            Port = port;
        }

        /// <summary>
        /// Получить расстояние в миллиметрах до точки
        /// </summary>
        /// <param name="channel">Номер канала по вертикали (от 0 до 15)</param>
        /// <param name="azimuth">Азимут точки</param>
        /// <returns>Возвращает расстояние в миллиметрах</returns>
        public int GetDisstance(int channel, double azimuth)
        {
            if (channel > 15 || channel < 0)
                throw new IndexOutOfRangeException("Channel number is greater than 15 or less than 0");
            NormalizeAngle(ref azimuth);

            return WaitDist(channel, azimuth);
        }

        /// <summary>
        /// Получить необработанные пакеты данных с лидара
        /// </summary>
        /// <param name="n">Количество пакетов</param>
        /// <returns>Возвращает массив массивов данных [n][1206]</returns>
        public byte[][] GetRawData(int n)
        {
            byte[][] data = new byte[n][];
            //Чтение данных по UDP
            UdpClient receivingUdpClient = new UdpClient(Port);
            IPEndPoint RemoteIpEndPoint = null;
            for (int i = 0; i < n; ++i)
            {
                data[i] = receivingUdpClient.Receive(ref RemoteIpEndPoint);
            }
            receivingUdpClient.Close();
            return data;
        }

        /// <summary>
        /// Получить один необработанный пакет данных с лидара
        /// </summary>
        /// <returns>Возвращает массив данных</returns>
        public byte[] GetRawPacket()
        {
            byte[] data;
            //Чтение данных по UDP
            UdpClient receivingUdpClient = new UdpClient(Port);
            //UdpClient receivingUdpClient = new UdpClient(Port);
            IPEndPoint RemoteIpEndPoint = null;
            data = receivingUdpClient.Receive(ref RemoteIpEndPoint);
            receivingUdpClient.Close();
            return data;
        }

        /// <summary>
        /// Получить данные с лидара об определённом секторе
        /// </summary>
        /// <param name="azimuthFrom">Угол начала сектора в градусах</param>
        /// <param name="azimuthTo">Угол конца сектора в градусах</param>
        /// <returns>Возвращает данные с лидара, упакованные в List</returns>
        public List<LidarDataBlock> GetData(double azimuthFrom, double azimuthTo)
        {
            NormalizeAngle(ref azimuthFrom);
            NormalizeAngle(ref azimuthTo);
            List<LidarDataBlock> LidarFormatedData = new List<LidarDataBlock>();
            UdpClient receivingUdpClient = new UdpClient(Port);
            IPEndPoint RemoteIpEndPoint = null;
            double angle; //Текущий азимут
            double oldAngle; //Предыдущий азимут
            bool recording = false; //Флаг записи данных
            bool firstItterFlag = true; //Флаг показывает, что обрабатывается первый пакет данных
            //Чтение данных по UDP
            lidarData = receivingUdpClient.Receive(ref RemoteIpEndPoint);
            if (lidarData.Length != 1206)
            {
                receivingUdpClient.Close();
                throw new DataMisalignedException("Incorrect length");
            }
            int pos = 2;
            angle = CalcAzimuth(GetByte(pos), GetByte(pos + 1));
            int i;
            while (true)
            {
                if (!firstItterFlag) //Если проверяется не первый массив данных
                {
                    pos = 2;
                    i = 0;
                }
                else
                {
                    pos = 102;
                    i = 1;
                    firstItterFlag = false;
                }
                //Сканирование массива данных
                for (; i < 12; ++i)
                {
                    oldAngle = angle;
                    if (oldAngle >= 360)
                    {
                        oldAngle -= 360;
                    }
                    angle = CalcAzimuth(GetByte(pos), GetByte(pos + 1));

                    //Решение проблемы со скачком при переходе через 360
                    if (oldAngle > angle) //Переход через 360
                    {
                        if (!recording && azimuthFrom >= 180 || recording && azimuthTo >= 180)
                        {
                            angle += 360;
                        }
                        else
                        {
                            oldAngle -= 360;
                        }
                    }
                    if (!recording)
                    {
                        if (azimuthFrom > oldAngle && azimuthFrom <= angle)
                        {
                            recording = true;
                        }
                    }
                    if (recording)
                    {
                        if (azimuthTo > oldAngle && azimuthTo <= angle)
                        {
                            lidarData = null;
                            receivingUdpClient.Close();
                            return LidarFormatedData;
                        }
                        LidarDataBlock block = new LidarDataBlock(lidarData, i);
                        LidarFormatedData.Add(block);
                    }
                    pos += 100;
                }
                //Чтение данных по UDP
                lidarData = receivingUdpClient.Receive(ref RemoteIpEndPoint);
                if (lidarData.Length != 1206)
                {
                    lidarData = null;
                    prevLidarData = null;
                    receivingUdpClient.Close();
                    throw new DataMisalignedException("Incorrect length");
                }
            }
        }

        /// <summary>
        /// Найти точку, до которой дальность максимальна
        /// </summary>
        /// <param name="dist">Найденная дальность</param>
        /// <param name="azimuth">Азимут точки</param>
        /// <param name="channel">Канал</param>
        public void FindMaximum(out int dist, out double azimuth, out int channel)
        {
            double azimuthFrom = 0;
            double azimuthTo = 359.99;
            int[] channelsList = new int[16];
            for (int i = 0; i < channelsList.Length; ++i)
            {
                channelsList[i] = i;
            }
            FindExtremum(out dist, out azimuth, out channel,
                true, azimuthFrom, azimuthTo, channelsList);
        }

        /// <summary>
        /// Найти точку, до которой дальность максимальна
        /// </summary>
        /// <param name="dist">Найденная дальность</param>
        /// <param name="azimuth">Азимут точки</param>
        /// <param name="channel">Канал</param>
        /// <param name="azimuthFrom">Угол начала сектора в градусах</param>
        /// <param name="azimuthTo">Угол конца сектора в градусах</param>
        public void FindMaximum(out int dist, out double azimuth, out int channel,
            double azimuthFrom, double azimuthTo)
        {
            int[] channelsList = new int[16];
            for (int i = 0; i < channelsList.Length; ++i)
            {
                channelsList[i] = i;
            }
            FindExtremum(out dist, out azimuth, out channel,
                true, azimuthFrom, azimuthTo, channelsList);
        }

        /// <summary>
        /// Найти точку, до которой дальность максимальна
        /// </summary>
        /// <param name="dist">Найденная дальность</param>
        /// <param name="azimuth">Азимут точки</param>
        /// <param name="channel">Канал</param>
        /// <param name="azimuthFrom">Угол начала сектора в градусах</param>
        /// <param name="azimuthTo">Угол конца сектора в градусах</param>
        /// <param name="channelsList">Каналы, на которых осуществляется поиск</param>
        public void FindMaximum(out int dist, out double azimuth, out int channel,
            double azimuthFrom, double azimuthTo, int[] channelsList)
        {
            if (channelsList == null)
            {
                throw new NullReferenceException("List of channels is null");
            }
            for (int i = 0; i < channelsList.Length; ++i)
            {
                if (channelsList[i] > 15 || channelsList[i] < 0)
                {
                    throw new IndexOutOfRangeException("List of channels contains incorrect channel number");
                }
            }
            FindExtremum(out dist, out azimuth, out channel,
                true, azimuthFrom, azimuthTo, channelsList);
        }

        /// <summary>
        /// Найти точку, до которой дальность минимальна
        /// </summary>
        /// <param name="dist">Найденная дальность</param>
        /// <param name="azimuth">Азимут точки</param>
        /// <param name="channel">Канал</param>
        public void FindMinimum(out int dist, out double azimuth, out int channel)
        {
            double azimuthFrom = 0;
            double azimuthTo = 359.99;
            int[] channelsList = new int[16];
            for (int i = 0; i < channelsList.Length; ++i)
            {
                channelsList[i] = i;
            }
            FindExtremum(out dist, out azimuth, out channel,
                false, azimuthFrom, azimuthTo, channelsList);
        }

        /// <summary>
        /// Найти точку, до которой дальность минимальна
        /// </summary>
        /// <param name="dist">Найденная дальность</param>
        /// <param name="azimuth">Азимут точки</param>
        /// <param name="channel">Канал</param>
        /// <param name="azimuthFrom">Угол начала сектора в градусах</param>
        /// <param name="azimuthTo">Угол конца сектора в градусах</param>
        public void FindMinimum(out int dist, out double azimuth, out int channel,
            double azimuthFrom, double azimuthTo)
        {
            int[] channelsList = new int[16];
            for (int i = 0; i < channelsList.Length; ++i)
            {
                channelsList[i] = i;
            }
            FindExtremum(out dist, out azimuth, out channel,
                false, azimuthFrom, azimuthTo, channelsList);
        }

        /// <summary>
        /// Найти точку, до которой дальность минимальна
        /// </summary>
        /// <param name="dist">Найденная дальность</param>
        /// <param name="azimuth">Азимут точки</param>
        /// <param name="channel">Канал</param>
        /// <param name="azimuthFrom">Угол начала сектора в градусах</param>
        /// <param name="azimuthTo">Угол конца сектора в градусах</param>
        /// <param name="channelsList">Каналы, на которых осуществляется поиск</param>
        public void FindMinimum(out int dist, out double azimuth, out int channel,
            double azimuthFrom, double azimuthTo, int[] channelsList)
        {
            if (channelsList == null)
            {
                throw new NullReferenceException("List of channels is null");
            }
            for (int i = 0; i < channelsList.Length; ++i)
            {
                if (channelsList[i] > 15 || channelsList[i] < 0)
                {
                    throw new IndexOutOfRangeException("List of channels contains incorrect channel number");
                }
            }
            FindExtremum(out dist, out azimuth, out channel,
                false, azimuthFrom, azimuthTo, channelsList);
        }

        //Статические методы

        /// <summary>
        /// Вычислить значение азимута по двум байтам
        /// </summary>
        /// <param name="a1">Первый байт (левый)</param>
        /// <param name="a2">Второй байт (правый)</param>
        /// <returns>Возвращает угол в градусах</returns>
        public static double CalcAzimuth(byte a1, byte a2)
        {
            int az = a2;
            az = az << 8;
            az += a1;
            return az / 100.0;
        }

        /// <summary>
        /// Вычислить значение дальности в миллиметрах по двум байтам
        /// </summary>
        /// <param name="a1">Первый байт (левый)</param>
        /// <param name="a2">Второй байт (правый)</param>
        /// <returns>Возвращает дальность в миллиметрах</returns>
        public static int CalcDist(byte a1, byte a2)
        {
            int d = a2;
            d = d << 8;
            d += a1;
            return d * 2;
        }

        /// <summary>
        /// Вычисление координат точки в прямоугольной системе
        /// </summary>
        /// <param name="dist">Расстояние до точки</param>
        /// <param name="azimuth">Азимут в градусах</param>
        /// <param name="channel">Номер канала лидара</param>
        /// <returns></returns>
        public static double[] CalcXYZ(double dist, double azimuth, int channel)
        {
            double [] coord=new double [3];
            double w = VerticalAngles[channel] * Math.PI / 180; //Угол места в радианах
            double a= azimuth * Math.PI / 180; //Азимут в радианах
            coord[0] = dist * Math.Cos(w) * Math.Sin(a);
            coord[1] = dist * Math.Cos(w) * Math.Cos(a);
            coord[2] = dist * Math.Sin(w);
            return coord;
        }

        /// <summary>
        /// Выбор канала с наиболее близким вертикальным углом
        /// </summary>
        /// <param name="w">Величина вертикального угла в градусах</param>
        /// <returns>Возвращает номер канала</returns>
        public static int BestChannel(double w)
        {
            while (w < -180)
            {
                w += 360;
            }
            while (w >= 180)
            {
                w -= 360;
            }
            return IndexOfMinDistance(VerticalAngles,w);
        }

        /// <summary>
        /// Узнать вертикальный угол для канала
        /// </summary>
        /// <param name="channel">Номер канала по вертикали(от 0 до 15)</param>
        /// <returns>Возвращает угол в градусах</returns>
        public static double GetVerticalAngle(int channel)
        {
            if (channel > 15 || channel < 0)
                throw new IndexOutOfRangeException("Channel number is greater than 15 or less than 0");
            return VerticalAngles[channel];
        }


        //Приватные методы

        /// <summary>
        /// Найти точку, до которой дальность максимальна или минимальна
        /// </summary>
        /// <param name="dist">Найденная дальность</param>
        /// <param name="azimuth">Азимут точки</param>
        /// <param name="channel">Канал</param>
        /// <param name="findMaz">true - если ищется максимум</param>
        /// <param name="azimuthFrom">Угол начала сектора в градусах</param>
        /// <param name="azimuthTo">Угол конца сектора в градусах</param>
        /// <param name="channelsList">Каналы, на которых осуществляется поиск</param>
        private void FindExtremum(out int dist, out double azimuth, out int channel,
        bool findMax, double azimuthFrom, double azimuthTo, int[] channelsList)
        {
            List<LidarDataBlock> LidarFormatedData = this.GetData(azimuthFrom, azimuthTo);
            if (LidarFormatedData.Count == 0)
            {
                throw new ArgumentOutOfRangeException("Azimuth interval is too small");
            }
            LidarDataBlock block = LidarFormatedData.ElementAt(0);
            azimuth = azimuth = block.Azimuth;
            dist = block.Distance1[channelsList[0]];
            channel = channelsList[0];
            for (int i = 0; i < LidarFormatedData.Count; ++i)
            {
                block = LidarFormatedData.ElementAt(i);
                for (int j = 0; j < channelsList.Length; j++)
                {
                    if (block.Distance1[channelsList[j]] > dist && findMax
                        || block.Distance1[channelsList[j]] < dist && !findMax)
                    {
                        azimuth = block.Azimuth;
                        dist = block.Distance1[channelsList[j]];
                        channel = channelsList[j];
                    }
                }
                for (int j = 0; j < block.Distance2.Length; j++)
                {
                    if (block.Distance2[channelsList[j]] > dist && findMax
                        || block.Distance2[channelsList[j]] < dist && !findMax)
                    {
                        if (i < LidarFormatedData.Count - 1)
                        {
                            LidarDataBlock nextBlock = LidarFormatedData.ElementAt(i + 1);
                            double azimuth1 = block.Azimuth;
                            double azimuth2 = nextBlock.Azimuth;
                            if (azimuth2 < azimuth1)
                            {
                                azimuth2 += 360;
                            }
                            azimuth = (block.Azimuth + nextBlock.Azimuth) / 2;
                        }
                        else
                        {
                            LidarDataBlock firstBlock = LidarFormatedData.ElementAt(0);
                            double deltaAzimuth = 0.5; //Шаг азимута
                            if (LidarFormatedData.Count > 1)
                            {
                                LidarDataBlock secondBlock = LidarFormatedData.ElementAt(1);
                                double azimuth1 = firstBlock.Azimuth;
                                double azimuth2 = secondBlock.Azimuth;
                                if (azimuth2 < azimuth1)
                                {
                                    azimuth2 += 360;
                                }
                                deltaAzimuth = (azimuth2 - azimuth1) / 2;
                            }
                            azimuth+=deltaAzimuth;
                        }
                        dist = block.Distance1[channelsList[j]];
                        channel = channelsList[j];
                        NormalizeAngle(ref azimuth);
                    }
                }
            }
        }

        /// <summary>
        /// Подключиться к лидару по UDP и ждать, пока не будет найден нужный азимут
        /// </summary>
        /// <param name="channel">Номер канала по вертикали (от 0 до 15)</param>
        /// <param name="azimuth">Азимут точки</param>
        /// <returns>Возвращает расстояние в миллиметрах</returns>
        private int WaitDist(int channel, double azimuth)
        {
            double angle;
            double oldAngle;

            //Чтение данных по UDP
            UdpClient receivingUdpClient = new UdpClient(Port);
            IPEndPoint RemoteIpEndPoint = null;
            lidarData = receivingUdpClient.Receive(ref RemoteIpEndPoint);
            if (lidarData.Length != 1206)
            {
                receivingUdpClient.Close();
                throw new DataMisalignedException("Incorrect length");
            }
            int pos = 2; //Индекс проверяемого байта данных
            angle = CalcAzimuth(GetByte(pos), GetByte(pos + 1));
            bool firstItterFlag = true; //Флаг показывает, что обрабатывается первый пакет данных
            while (true)
            {
                if (!firstItterFlag) //Если проверяется не первый массив данных
                {
                    pos = -98;
                }
                else
                {
                    firstItterFlag = false;
                }
                //Сканирование массива данных
                for (; pos <= 1200; pos += 100)
                {
                    oldAngle = angle;
                    if (oldAngle >= 360)
                    {
                        oldAngle -= 360;
                    }
                    angle = CalcAzimuth(GetByte(pos), GetByte(pos + 1));
                    //Решение проблемы со скачком при переходе через 360
                    if (oldAngle > angle) //Переход через 360
                    {
                        if (azimuth >= 180)
                        {
                            angle += 360;
                        }
                        else
                        {
                            oldAngle -= 360;
                        }
                    }
                    if (azimuth >= oldAngle && azimuth <= angle)
                    {
                        double[] angles = { oldAngle, (oldAngle + angle) / 2, angle };
                        int minIndex = IndexOfMinDistance(angles, azimuth);
                        int distPos = pos + 2 + channel * 3;
                        switch (minIndex)
                        {
                            case 0:
                                distPos -= 100;
                                break;
                            case 1:
                                distPos -= 52;
                                break;
                        }
                        int dist=CalcDist(GetByte(distPos), GetByte(distPos + 1)); //Вычисление дистанции
                        receivingUdpClient.Close();
                        lidarData = null;
                        prevLidarData = null;
                        return dist;
                    }
                    prevLidarData = lidarData;
                    //Чтение данных по UDP
                    lidarData = receivingUdpClient.Receive(ref RemoteIpEndPoint);
                    if (lidarData.Length != 1206)
                    {
                        receivingUdpClient.Close();
                        throw new DataMisalignedException("Incorrect length");
                    }
                }

            }
        }

        /// <summary>
        /// Выбирает байт из последнего или предпоследнего массивов
        /// </summary>
        /// <param name="pos">Индекс байта (положительный - из последнего массива,
        /// отрицательный - из предпоследнего)</param>
        /// <returns>Возвращает байт данных</returns>
        private byte GetByte(int pos)
        {
            if (pos >= 0)
            {
                if (lidarData == null)
                    throw new NullReferenceException("No data available");
                if (pos >= lidarData.Length)
                    throw new IndexOutOfRangeException("Byte index is out of range");
                return lidarData[pos];
            }
            else
            {
                const int serviceBytes = 6; //Число спец. байтов в конце массива
                if (prevLidarData == null)
                    throw new NullReferenceException("No data available");
                if (-pos > prevLidarData.Length - serviceBytes)
                    throw new IndexOutOfRangeException("Byte index is out of range");

                return prevLidarData[prevLidarData.Length - serviceBytes + pos];
            }
        }

        /// <summary>
        /// Получить номер элемента массива, который имеет минимальную разницу с x
        /// </summary>
        /// <param name="array">Массив</param>
        /// <param name="x">Элемент</param>
        /// <returns>Возвращает номер элемента массива</returns>
        private static int IndexOfMinDistance(double [] array, double x)
        {
            double minDif = 999; //Разница между азимутом и исследуемым углом
            int minIndex = 0;
            for (int j = 0; j < array.Length; ++j) //Поиск минимальной разницы
            {
                double dif = Math.Abs(array[j] - x);
                if (j == 0 || dif < minDif)
                {
                    minDif = dif;
                    minIndex = j;
                }
            }
            return minIndex;
        }


        /// <summary>
        /// Представить угол в диапазоне от 0 до 360 градусов
        /// </summary>
        /// <param name="angle">Исходное значение угла</param>
        /// <returns>Возвращает значение угла в диапазоне от 0 до 360 градусов</returns>
        private void NormalizeAngle(ref double angle)
        {
            while (angle < 0)
            {
                angle += 360;
            }
            while (angle >= 360)
            {
                angle -= 360;
            }
        }
    }
}
