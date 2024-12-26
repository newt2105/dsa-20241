#ifndef ULTIS_H
#define ULTIS_H

#include <iostream>
#include <vector>
#include <iterator>
#include <iomanip>
#include <numeric>
#include <algorithm>
#include <memory>
#include <sstream>
#include <forward_list>

using namespace std;

const string DATA_FILE_PATH = "/home/dihnhuunam/Workspace/dsa-20241/data.txt";

enum class Align
{
	Left,
	Right,
	Center
};
typedef forward_list<string> Row;

class ConsoleTable
{
public:
	ConsoleTable() = delete;
	~ConsoleTable() = default;
	ConsoleTable(unsigned int numberOfColumns);

public:
	void WriteTable(Align align = Align::Left, ostream *stream = &cout) const;
	void AddNewRow(const forward_list<string> &list);

private:
	void GenerateStream(stringstream &, Align align, int i, const vector<int> &columnsWidth) const;
	string AlignRowToLeftOrRight(Align align, int index, const vector<int> &columnsWidth) const;
	string AlignRowToCenter(int index, const vector<int> &columnsWidth) const;
	void WriteBorderToStream(int width, stringstream *stream) const;
	vector<int> GetColumnsMaxWidth() const;

private:
	unsigned int _numberOfColumns;
	vector<vector<string>> _rows;
};
#endif
