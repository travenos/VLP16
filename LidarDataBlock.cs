using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace LidarVLP16
{
    /// <summary>
    /// Один блок данных с лидара (азимут;расстояния с 16-ти каналов; степени отражения
    /// с 16-ти каналов; расстояния и степени отражения с 16-ти каналов следующего азимута)
    /// </summary>
    class LidarDataBlock
    {
        /// <summary>
        /// Азимут
        /// </summary>
        public double Azimuth;
        /// <summary>
        /// Расстояния на каждом канале до точек первого азимута
        /// </summary>
        public readonly int[] Distance1 = new int[16];
        /// <summary>
        /// Степени отражения на каждом канале от точек первого азимута
        /// </summary>
        public readonly byte[] Reflectivity1 = new byte[16];
        /// <summary>
        /// Расстояния на каждом канале до точек соседнего азимута
        /// </summary>
        public readonly int[] Distance2 = new int[16];
        /// <summary>
        /// Степени отражения на каждом канале от точек соседнего азимута
        /// </summary>
        public readonly byte[] Reflectivity2 = new byte[16];

        /// <summary>
        /// Записать в объект один блок данных с лидара
        /// </summary>
        /// <param name="lidarData">Необработанные данные с лидара</param>
        /// <param name="blockNumber">Номер блока данных</param>
        public LidarDataBlock(byte[] lidarData, int blockNumber)
        {
            if (lidarData.Length!=1206 || blockNumber>11 || blockNumber < 0)
            {
                throw new IndexOutOfRangeException("Incorrect input data");
            }
            int pos = blockNumber*100 + 2; //Текущий индекс в массиве данных
            Azimuth = Lidar.CalcAzimuth(lidarData[pos], lidarData[pos + 1]);
            pos += 2;
            int i;
            for (i=0; i<16; ++i)
            {
                Distance1[i] = Lidar.CalcDist(lidarData[pos], lidarData[pos + 1]);
                pos += 2;
                Reflectivity1[i] = lidarData[pos];
                pos++;
            }
            for (i = 0; i < 16; ++i)
            {
                Distance2[i] = Lidar.CalcDist(lidarData[pos], lidarData[pos + 1]);
                pos += 2;
                Reflectivity2[i] = lidarData[pos];
                pos++;
            }
        }
        
        /// <summary>
        /// Представить блок данных в виде текста
        /// </summary>
        /// <returns>Возвращает многострочный текст</returns>
        public string ToText()
        {
            string str="Azimuth="+Convert.ToString(Azimuth)+" grad: \n";
            int i;
            for (i = 0; i < 16; ++i)
            {
                str+="Channel "+Convert.ToString(i) + ": ";
                str+="distance="+Convert.ToString(Distance1[i]) + " mm, ";
                str += "reflectivity=" + Convert.ToString(Reflectivity1[i]) + ";\n";
            }
            str += "Next azimuth:\n";
            for (i = 0; i < 16; ++i)
            {
                str += "Channel " + Convert.ToString(i) + ": ";
                str += "distance=" + Convert.ToString(Distance2[i]) + " mm, ";
                str += "reflectivity=" + Convert.ToString(Reflectivity2[i]) + ";\n";
            }
            return str;
        }
    }
}
