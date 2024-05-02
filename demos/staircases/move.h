#ifndef CARTOCROW_MOVE_H
#define CARTOCROW_MOVE_H

#include "types.h"
#include "staircase.h"

class Command {
  public:
	virtual void execute() = 0;
	virtual void undo() = 0;
	virtual ~Command() = default;
};

class Contraction : public Command {
  public:
	Contraction(std::shared_ptr<Staircase> staircase, MoveBox box);
	void execute() override;
	void undo() override;

  private:
	std::shared_ptr<Staircase> m_staircase;
	MoveBox m_box;
	std::optional<Number<K>> m_erased_x;
	std::optional<Number<K>> m_erased_y;
};

class EdgeMove : public Command {
  public:
	EdgeMove(std::shared_ptr<Staircase> staircase, Edge edge, Number<K> start_pos, Number<K> new_pos);
	void execute() override;
	void undo() override;

  private:
	std::shared_ptr<Staircase> m_staircase;
	Edge m_edge;
	Number<K> m_new_pos;
	Number<K> m_start_pos;
};

std::unique_ptr<Command> move_or_contract(const std::shared_ptr<Staircase>& staircase, Edge edge, Number<K> start_pos, Number<K> new_pos);

#endif //CARTOCROW_MOVE_H
