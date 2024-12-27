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

/**
 * @brief Path to the data file.
 */
extern const string DATA_FILE_PATH;

/**
 * @enum Align
 * @brief Enum for text alignment in table columns.
 */
enum class Align
{
	Left,
	Right,
	Center
};

/**
 * @typedef Row
 * @brief Alias for a forward_list of strings representing a row in the table.
 */
typedef forward_list<string> Row;

/**
 * @class ConsoleTable
 * @brief Class for creating and displaying formatted tables in the console.
 */
class ConsoleTable
{
public:
	/**
	 * @brief Deleted default constructor.
	 */
	ConsoleTable() = delete;

	/**
	 * @brief Destructor (defaulted).
	 */
	~ConsoleTable() = default;

	/**
	 * @brief Constructor to initialize the table with a specified number of columns.
	 * @param numberOfColumns Number of columns in the table.
	 */
	ConsoleTable(unsigned int numberOfColumns);

	/**
	 * @brief Writes the table to the console or specified output stream.
	 * @param align The alignment for text in the table columns (default: Align::Left).
	 * @param stream The output stream to write to (default: std::cout).
	 */
	void WriteTable(Align align = Align::Left, ostream *stream = &cout) const;

	/**
	 * @brief Adds a new row to the table.
	 * @param list A forward_list of strings representing the row data.
	 */
	void AddNewRow(const forward_list<string> &list);

private:
	/**
	 * @brief Generates a row stream with aligned content.
	 * @param stream The stream to write the row to.
	 * @param align The alignment for the row.
	 * @param i The row index.
	 * @param columnsWidth A vector containing the maximum widths for each column.
	 */
	void GenerateStream(stringstream &stream, Align align, int i, const vector<int> &columnsWidth) const;

	/**
	 * @brief Aligns a row to the left or right.
	 * @param align The alignment for the row (Align::Left or Align::Right).
	 * @param index The row index.
	 * @param columnsWidth A vector containing the maximum widths for each column.
	 * @return A string representing the aligned row.
	 */
	string AlignRowToLeftOrRight(Align align, int index, const vector<int> &columnsWidth) const;

	/**
	 * @brief Aligns a row to the center.
	 * @param index The row index.
	 * @param columnsWidth A vector containing the maximum widths for each column.
	 * @return A string representing the center-aligned row.
	 */
	string AlignRowToCenter(int index, const vector<int> &columnsWidth) const;

	/**
	 * @brief Writes a border line to the stream.
	 * @param width The width of the border line.
	 * @param stream The stream to write the border to.
	 */
	void WriteBorderToStream(int width, stringstream *stream) const;

	/**
	 * @brief Calculates the maximum width for each column.
	 * @return A vector containing the maximum widths for each column.
	 */
	vector<int> GetColumnsMaxWidth() const;

private:
	unsigned int _numberOfColumns;
	vector<vector<string>> _rows;
};

#endif
