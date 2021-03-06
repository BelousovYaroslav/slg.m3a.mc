#ifndef T39_M3A_ANALOGUE_PARAMETERS_LIST
#define T39_M3A_ANALOGUE_PARAMETERS_LIST

//��������� ������������ �� ID
#define UTD1                0   //0x00        ����������� 1
#define UTD2                1   //0x01        ����������� 2
#define UTD3                2   //0x02        ����������� 3
#define I1                  3   //0x03        ��������� ��� i1
#define I2                  4   //0x04        ��������� ��� i2
#define CNTRPC              5   //0x05        ���������� �� ����������������
#define AMPLANG_ALTERA      6   //0x06        ��������� ���������� �� alter'�
#define AMPLANG_DUS         7   //0x07        ��������� ���������� � ����
#define RULA                8   //0x08        ���������� RULA
#define AMPLITUDE           9   //0x09        �������� ��������� ���������

#define TACT_CODE           10  //0x0A        ��� ����� ���������
#define M_COEFF             11  //0x0B        ����������� M
#define STARTMODE           12  //0x0C        ��������� ����
#define DECCOEFF_CURRENT    13  //0x0D        ������� ����������� ������
#define CONTROL_I1          14  //0x0E        ����������� ��� ������� I1
#define CONTROL_I2          15  //0x0F        ����������� ��� ������� I2
#define CONTROL_AA          16  //0x10        ����������� ������ �������� AmplAng
#define HV_APPLY_COUNT_SET  17  //0x11        �������� �������. �������� ���-�� HV ������
#define HV_APPLY_COUNT_TR   18  //0x12        �������� �������. ���������� HV ������ � ���� �������
#define HV_APPLY_DURAT_SET  19  //0x13        �������� �������. �������� ������������ HV ������

#define SIGNCOEFF           20  //0x14        �������� �����������
#define DEVNUM              21  //0x15        ����� �������
//#define DEVNUM              22  //0x16        ����� �������. ������� ����
#define DATE_Y              23  //0x17        ����.���
#define DATE_M              24  //0x18        ����.�����
#define DATE_D              25  //0x19        ����.����
#define ORG                 26  //0x1A        ����������� ������� (��� ������� �������)
#define ORG_B1              27  //0x1B        �����������.����1
#define ORG_B2              28  //0x1C        �����������.����2
#define ORG_B3              29  //0x1D        �����������.����3

#define ORG_B4              30  //0x1E        �����������.����4
#define ORG_B5              31  //0x1F        �����������.����5
#define ORG_B6              32  //0x20  " "   �����������.����6
#define ORG_B7              33  //0x21  "!"   �����������.����7
#define ORG_B8              34  //0x22  """   �����������.����8
#define ORG_B9              35  //0x23  "#"   �����������.����9
#define ORG_B10             36  //0x24  "$"   �����������.����10
#define ORG_B11             37  //0x25  "%"   �����������.����11
#define ORG_B12             38  //0x26  "&"   �����������.����12
#define ORG_B13             39  //0x27  "'"   �����������.����13

#define ORG_B14             40  //0x28  "("   �����������.����14
#define ORG_B15             41  //0x29  ")"   �����������.����15
#define ORG_B16             42  //0x2A  "*"   �����������.����16 ��� ������������ 0 �� �����!
#define VERSION             43  //0x2B  "+"   ������
#define CALIB_T1            44  //0x2C  ","   ����������� ������ ������������� ����� ����������
#define T1_TD1              45  //0x2D  "-"   ��������� ������� TD1 �� ������ ����. �����
#define T1_TD2              46  //0x2E  "."   ��������� ������� TD2 �� ������ ����. �����
#define T1_TD3              47  //0x2F  "/"   ��������� ������� TD3 �� ������ ����. �����
#define CALIB_T2            48  //0x30  "0"   ����������� ������� ������������� ����� ����������
#define T2_TD1              49  //0x31  "1"   ��������� ������� TD1 �� ������� ����. �����

#define T2_TD2              50  //0x32  "2"   ��������� ������� TD2 �� ������� ����. �����
#define T2_TD3              51  //0x33  "3"   ��������� ������� TD3 �� ������� ����. �����
#define HV_APPLY_PACKS      52  //0x34  "4"   �������� �������. ���������� ����� 3kV ��������� �������

#endif