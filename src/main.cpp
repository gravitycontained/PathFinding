#include <qpl/qpl.hpp>
#include <queue>

template<bool allow_diagonal, typename T>
struct bfs_visualized {
	std::queue<std::shared_ptr<qpl::bfs_node>> queue;
	std::unordered_set<qpl::vec2s> visited;
	std::shared_ptr<qpl::bfs_node> current_node;
	bool finished = false;

	constexpr static auto direction = [&]() {
		if constexpr (allow_diagonal) {
			return std::array{
				qpl::vec2is{1, 0}, qpl::vec2is(0, 1), qpl::vec2is(-1, 0), qpl::vec2is(0, -1),
				qpl::vec2is(1, 1), qpl::vec2is(1, -1), qpl::vec2is(-1, 1), qpl::vec2is(-1, -1) };
		}
		else {
			return std::array{ qpl::vec2is{1, 0}, qpl::vec2is(0, 1), qpl::vec2is(-1, 0), qpl::vec2is(0, -1) };
		}
	}();

	void reset() {
		std::queue<std::shared_ptr<qpl::bfs_node>> empty;
		this->queue.swap(empty);
		this->visited.clear();
		this->finished = false;
	}
	void prepare(const std::vector<std::vector<T>>& maze, qpl::vec2s start, qpl::vec2s end) {
		auto starting_node = std::make_shared<qpl::bfs_node>(start, nullptr);
		this->queue.push(starting_node);
		this->visited.insert(start);
	}

	auto get_path() const {
		std::vector<qpl::vec2s> path;
		if (this->queue.empty()) {
			return path;
		}
		auto traverse = this->current_node;
		while (traverse) {
			path.push_back(traverse->position);
			traverse = traverse->parent;
		}
		std::ranges::reverse(path);
		return path;
	}

	void step(const std::vector<std::vector<T>>& maze, qpl::vec2s start, qpl::vec2s end, qpl::size repeat) {
		for (qpl::size i = 0u; i < repeat; ++i) {
			if (this->finished) {
				return;
			}
			if (this->queue.empty()) {
				return;
			}
			this->current_node = this->queue.front();
			this->queue.pop();

			auto width = qpl::signed_cast(maze[0].size());
			auto height = qpl::signed_cast(maze.size());

			if (current_node->position == end) {
				qpl::println("at the end!");
				this->finished = true;
				return;
			}
			for (auto& dir : this->direction) {
				auto check = qpl::vec2is(this->current_node->position) + dir;

				if (check.x >= 0 && check.x < width && check.y >= 0 && check.y < height) {
					bool valid = maze[check.y][check.x] != 1;
					if (valid) {
						bool found = false;
						for (auto& i : this->visited) {
							if (i == check) {
								found = true;
								break;
							}
						}
						if (!found) {
							auto next = std::make_shared<qpl::bfs_node>(check, this->current_node);
							this->queue.push(next);
							this->visited.insert(check);
						}
					}
				}
			}
		}
	}
};

template<typename T, bool diagonal = false>
struct astar_visualized {

	std::vector<std::shared_ptr<qpl::astar_node>> queue;
	std::unordered_set<qpl::vec2s> visited;
	std::shared_ptr<qpl::astar_node> current_node;
	bool finished = false;

	constexpr static auto direction = [&]() {
		if constexpr (diagonal) {
			return std::array{
				qpl::vec2is{1, 0}, qpl::vec2is(0, 1), qpl::vec2is(-1, 0), qpl::vec2is(0, -1),
				qpl::vec2is(1, 1), qpl::vec2is(1, -1), qpl::vec2is(-1, 1), qpl::vec2is(-1, -1) };
		}
		else {
			return std::array{ qpl::vec2is{1, 0}, qpl::vec2is(0, 1), qpl::vec2is(-1, 0), qpl::vec2is(0, -1) };
		}
	}();

	void reset() {
		this->queue.clear();
		this->visited.clear();
		this->finished = false;
	}
	void prepare(const std::vector<std::vector<T>>& maze, qpl::vec2s start, qpl::vec2s end) {
		std::shared_ptr<qpl::astar_node> starting_node = std::make_shared<qpl::astar_node>(start, nullptr);
		this->queue.push_back(starting_node);
		this->visited.insert(start);
	}

	auto get_path() const {
		std::vector<qpl::vec2s> path;
		if (this->queue.empty()) {
			return path;
		}
		auto traverse = this->current_node;
		while (traverse) {
			path.push_back(traverse->position);
			traverse = traverse->parent;
		}
		std::ranges::reverse(path);
		return path;
	}

	void step(const std::vector<std::vector<T>>& maze, qpl::vec2s start, qpl::vec2s end) {
		if (this->finished) {
			return;
		}
		if (this->queue.empty()) {
			return;
		}
		this->current_node = *this->queue.begin();
		qpl::size current_index = 0u;
		qpl::size ctr = 0u;
		for (auto& element : queue) {
			if (element->f < this->current_node->f) {
				this->current_node = element;
				current_index = ctr;
			}
			++ctr;
		}

		this->visited.insert(this->current_node->position);
		this->queue.erase(this->queue.begin() + current_index);

		auto width = qpl::signed_cast(maze[0].size());
		auto height = qpl::signed_cast(maze.size());

		if (this->current_node->position == end) {
			qpl::println("at the end!");
			this->finished = true;
			return;
		}

		std::vector<std::shared_ptr<qpl::astar_node>> children;
		for (auto& dir : direction) {
			auto check = qpl::vec2is(this->current_node->position) + dir;

			if (check.x >= 0 && check.x < width && check.y >= 0 && check.y < height) {
				bool valid = maze[check.y][check.x] != 1;
				if (valid) {
					bool found = this->visited.find(check) != this->visited.cend();

					bool queue_found = false;
					for (auto& i : queue) {
						if (i->position == check) {
							queue_found = true;
							break;
						}
					}

					if (!found && !queue_found) {
						auto next = std::make_shared<qpl::astar_node>(check, this->current_node);
						children.push_back(next);
					}
				}
			}
		}

		for (auto& child : children) {
			auto x = qpl::signed_cast(child->position.x) - qpl::signed_cast(end.x);
			auto y = qpl::signed_cast(child->position.y) - qpl::signed_cast(end.y);

			child->g = this->current_node->g + 1u;
			child->h = x * x + y * y;
			child->f = child->g + child->h;

			this->queue.push_back(child);
		}
	}
};

struct maze {
	std::vector<std::vector<qpl::size>> cells;
	qpl::vec2s dimension;

	auto& get(qpl::size x, qpl::size y) {
		return this->cells[y][x];
	}

	void create(qpl::vec2s dimension) {
		this->dimension = dimension;
		this->cells.resize(this->dimension.y);
		for (auto& i : this->cells) {
			i.resize(this->dimension.x);
		}
	}
};

struct maze_graphic {
	qsf::vertex_array va;
	qsf::rectangles circles;
	maze before;
	qpl::vec2s dimension;
	constexpr static auto cell_dim = qpl::vec(25, 25);

	void create(const maze& maze) {
		this->dimension = maze.dimension;
		this->va.resize(this->dimension.x * this->dimension.y * 4);
		this->va.set_primitive_type(qsf::primitive_type::quads);

		qpl::size ctr = 0u;
		for (auto [x, y] : this->dimension.list_possibilities_range()) {
			auto pos = qpl::vec(x, y);

			this->va[ctr++].position = (pos + qpl::vec(0, 0)) * this->cell_dim;
			this->va[ctr++].position = (pos + qpl::vec(1, 0)) * this->cell_dim;
			this->va[ctr++].position = (pos + qpl::vec(1, 1)) * this->cell_dim;
			this->va[ctr++].position = (pos + qpl::vec(0, 1)) * this->cell_dim;
		}

		this->before.create(this->dimension);
		for (auto& i : this->before.cells) {
			for (auto& i : i) {
				i = qpl::size_max;
			}
		}
	}

	void update(const maze& maze) {
		if (this->dimension != maze.dimension) {
			this->create(maze);
		}

		constexpr auto colors = std::array{ qpl::rgb(200, 200, 200), qpl::rgb(50, 50, 50) };
		for (auto [x, y] : this->dimension.list_possibilities_range()) {

			auto& b = this->before.cells[y][x];
			auto value = maze.cells[y][x];
			if (b != value) {
				b = value;

				this->set_color(x, y, colors[value]);
			}
		}
	}
	void add_path(const std::vector<qpl::vec2s>& path, qpl::rgb color) {
		for (auto& i : path) {
			qsf::rectangle rect;
			rect.set_dimension(this->cell_dim);
			rect.set_color(color);
			rect.set_position(this->cell_dim * i);
			this->circles.add(rect);
		}
	}

	auto& get(qpl::size x, qpl::size y, qpl::size i) {
		auto index = (y * this->dimension.x + x);
		return this->va[index * 4 + i];
	}

	void set_color(qpl::size x, qpl::size y, qpl::rgb color) {
		for (qpl::size i = 0u; i < 4u; ++i) {
			this->get(x, y, i).color = color;
		}
	}

	void draw(qsf::draw_object& draw) const {
		draw.draw(this->va);
		draw.draw(this->circles);
	}
};

struct main_state : qsf::base_state {
	void randomize() {

		qpl::perlin_noise perlin;
		perlin.set_seed_random();

		for (auto [x, y] : this->maze.dimension.list_possibilities_range()) {
			auto value = perlin.get(qpl::vec(x, y), 0.03, 2);
			this->maze.cells[y][x] = value < 0.45 ? 1 : 0;
		}
		
		auto n = 30;
		for (auto [x, y] : qpl::vec(n, n).list_possibilities_range()) {
			this->maze.cells[y][x] = 0;
		}
		for (auto [x, y] : qpl::vec(n, n).list_possibilities_range()) {
			auto pos = this->maze_size - 1 - qpl::vec(x, y);
			this->maze.cells[pos.y][pos.x] = 0;
		}
	}

	qpl::vec2s maze_size = qpl::vec(750, 750);
	qpl::f64 bfs_sum = 0.0;
	qpl::f64 astar_sum = 0.0;

	void solve() {

		auto valid_path = [](qpl::size a) {
			return a != 1;
		};

		qpl::clock clock;
		auto solve1 = qpl::bfs_path_finding<true>(this->maze.cells, { 0, 0 }, this->maze_size - 250, valid_path);
		if (solve1.empty()) {
			qpl::println("no path found");
			return;
		}
		this->bfs_sum += clock.elapsed_f();

		clock.reset();
		auto solve2 = qpl::astar_path_finding<true>(this->maze.cells, { 0, 0 }, this->maze_size - 250, valid_path);
		this->astar_sum += clock.elapsed_f();

		qpl::println("BFS ", qpl::secs(this->bfs_sum).string(), "\nA* ", qpl::secs(this->astar_sum).string());
		qpl::println();

		this->maze_graphic.circles.clear();
		this->maze_graphic.add_path(solve1, qpl::rgb::red().with_alpha(150));
		this->maze_graphic.add_path(solve2, qpl::rgb::blue().with_alpha(150));
	}

	void init() override {
		this->maze.create(maze_size);
		this->randomize();
		this->maze_graphic.update(this->maze);

		this->solve();
		this->path_finder_visualized.prepare(this->maze.cells, { 0, 0 }, this->maze_size - 250);

		this->view.set_hitbox(*this);
	}

	void updating() override {
		this->update(this->view);

		if (this->event().key_holding(sf::Keyboard::Space)) {
			this->path_finder_visualized.step(this->maze.cells, { 0, 0 }, this->maze_size - 250, qpl::size_cast(this->steps));

			this->maze_graphic.circles.clear();
			this->maze_graphic.add_path(this->path_finder_visualized.get_path(), qpl::rgb::red());

			if (this->lock) {
				this->view.set_center(this->path_finder_visualized.get_path().back() * maze_graphic::cell_dim);
			}
		}
		if (this->event().key_pressed(sf::Keyboard::A)) {
			this->steps *= 0.75;
			qpl::println(qpl::size_cast(this->steps), " steps");
		}
		else if (this->event().key_pressed(sf::Keyboard::D)) {
			this->steps *= (4.0 / 3);
			qpl::println(qpl::size_cast(this->steps), " steps");
		}
		else if (this->event().key_single_pressed(sf::Keyboard::L)) {
			this->lock = !this->lock;
		}
		else if (this->event().key_single_pressed(sf::Keyboard::S)) {
			this->solve();
		}
		else if (this->event().key_single_pressed(sf::Keyboard::R)) {
			this->randomize();
			this->path_finder_visualized.reset();
			this->path_finder_visualized.prepare(this->maze.cells, { 0, 0 }, this->maze_size - 250);
			this->maze_graphic.update(this->maze);

			this->maze_graphic.circles.clear();
			this->maze_graphic.add_path(this->path_finder_visualized.get_path(), qpl::rgb::red());
		}
	}
	void drawing() override {
		this->draw(this->maze_graphic, this->view);
	}

	maze maze;
	maze_graphic maze_graphic;
	qsf::view_rectangle view;
	//astar_visualized<qpl::size, true> path_finder_visualized;
	bfs_visualized<true, qpl::size> path_finder_visualized;
	qpl::f64 steps = 5.0;
	bool lock = false;
};

int main() try {

	qsf::framework framework;
	framework.set_title("QPL");
	framework.set_dimension({ 1400u, 950u });

	framework.add_state<main_state>();
	framework.game_loop();
}
catch (std::exception& any) {
	qpl::println("caught exception:\n", any.what());
	qpl::system_pause();
}