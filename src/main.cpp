#include <qpl/qpl.hpp>
#include <queue>

struct node {
	qpl::vec2s position;
	std::shared_ptr<node> parent;

	node() {

	}
	node(qpl::vec2s position, const std::shared_ptr<node>& parent) {
		this->position = position;
		this->parent = parent;
	}
	bool operator==(const node& other) const {
		return this->position == other.position;
	}

	bool operator<(const node& other) const {
		return this->position.x < other.position.x || (this->position.x == other.position.x && this->position.y < other.position.y);
	}
};
template<typename T, bool diagonal = false>
struct bfs_visualized {

	std::queue<std::shared_ptr<node>> queue;
	std::unordered_set<qpl::vec2s> visited;
	std::vector<qpl::vec2s> path;

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
		std::queue<std::shared_ptr<node>> empty;
		this->queue.swap(empty);
		this->visited.clear();
		this->path.clear();
	}
	void prepare(const std::vector<std::vector<T>>& maze, qpl::vec2s start, qpl::vec2s end) {
		std::shared_ptr<node> starting_node = std::make_shared<node>(start, nullptr);
		this->queue.push(starting_node);
		this->visited.insert(start);
	}

	auto get_path(std::shared_ptr<node>& current_node) const {
		std::vector<qpl::vec2s> path;
		auto traverse = current_node;
		while (traverse) {
			path.push_back(traverse->position);
			traverse = traverse->parent;
		}
		std::ranges::reverse(path);
		return path;
	}

	void step(const std::vector<std::vector<T>>& maze, qpl::vec2s start, qpl::vec2s end) {
		if (this->queue.empty()) {
			return;
		}
		std::shared_ptr<node> current_node = this->queue.front();
		this->queue.pop();

		auto width = qpl::signed_cast(maze[0].size());
		auto height = qpl::signed_cast(maze.size());

		if (current_node->position == end) {
			this->path = this->get_path(current_node);
			return;
		}
		for (auto& dir : this->direction) {
			auto check = qpl::vec2is(current_node->position) + dir;

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
						auto next = std::make_shared<node>(check, current_node);
						this->queue.push(next);
						this->visited.insert(check);
					}
				}
			}
		}
		this->path = this->get_path(current_node);
	}
};


template<typename T, bool diagonal = false>
struct astar_visualized {

	std::vector<std::shared_ptr<qpl::astar_node<qpl::size>>> queue;
	std::unordered_set<qpl::vec2s> visited;
	std::vector<qpl::vec2s> path;
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
		this->path.clear();
		this->finished = false;
	}
	void prepare(const std::vector<std::vector<T>>& maze, qpl::vec2s start, qpl::vec2s end) {
		std::shared_ptr<qpl::astar_node<qpl::size>> starting_node = std::make_shared<qpl::astar_node<qpl::size>>(start, nullptr);
		this->queue.push_back(starting_node);
		this->visited.insert(start);
	}

	auto get_path(std::shared_ptr<qpl::astar_node<qpl::size>>& current_node) const {
		std::vector<qpl::vec2s> path;
		auto traverse = current_node;
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
		std::shared_ptr<qpl::astar_node<qpl::size>> current_node = *queue.begin();
		qpl::size current_index = 0u;
		qpl::size ctr = 0u;
		for (auto& element : queue) {
			if (element->f < current_node->f) {
				current_node = element;
				current_index = ctr;
			}
			++ctr;
		}

		visited.insert(current_node->position);
		queue.erase(queue.begin() + current_index);

		auto width = qpl::signed_cast(maze[0].size());
		auto height = qpl::signed_cast(maze.size());

		if (current_node->position == end) {
			qpl::println("at the end!");
			this->finished = true;
			this->path = this->get_path(current_node);
			return;
		}

		std::vector<std::shared_ptr<qpl::astar_node<qpl::size>>> children;
		for (auto& dir : direction) {
			auto check = qpl::vec2is(current_node->position) + dir;

			if (check.x >= 0 && check.x < width && check.y >= 0 && check.y < height) {
				bool valid = maze[check.y][check.x] != 1;
				if (valid) {
					bool found = visited.find(check) != visited.cend();

					bool queue_found = false;
					for (auto& i : queue) {
						if (i->position == check) {
							queue_found = true;
							break;
						}
					}

					if (!found && !queue_found) {
						auto next = std::make_shared<qpl::astar_node<qpl::size>>(check, current_node);
						children.push_back(next);
					}
				}
			}
		}

		for (auto& child : children) {
			auto x = qpl::signed_cast(child->position.x) - qpl::signed_cast(end.x);
			auto y = qpl::signed_cast(child->position.y) - qpl::signed_cast(end.y);

			child->g = current_node->g + 1u;
			child->h = x * x + y * y;
			child->f = child->g + child->h;

			queue.push_back(child);
		}

		this->path = this->get_path(current_node);
	}
};


template<typename T, bool diagonal = true> requires (std::is_integral_v<T>)
std::vector<qpl::vec2s> breadth_first_search(const std::vector<std::vector<T>>& maze, qpl::vec2s start, qpl::vec2s end) {
	if (maze.empty()) {
		return{};
	}
	auto width = qpl::signed_cast(maze[0].size());
	auto height = qpl::signed_cast(maze.size());

	std::queue<std::shared_ptr<node>> queue;
	std::shared_ptr<node> starting_node = std::make_shared<node>(start, nullptr);
	queue.push(starting_node);

	std::unordered_set<qpl::vec2s> visited;
	visited.insert(start);

	constexpr auto direction = [&]() {
		if constexpr (diagonal) {
			return std::array{
				qpl::vec2is{1, 0}, qpl::vec2is(0, 1), qpl::vec2is(-1, 0), qpl::vec2is(0, -1),
				qpl::vec2is(1, 1), qpl::vec2is(1, -1), qpl::vec2is(-1, 1), qpl::vec2is(-1, -1) };
		}
		else {
			return std::array{ qpl::vec2is{1, 0}, qpl::vec2is(0, 1), qpl::vec2is(-1, 0), qpl::vec2is(0, -1) };
		}
	}();

	while (!queue.empty()) {
		std::shared_ptr<node> current_node = queue.front();
		queue.pop();

		if (current_node->position == end) {
			auto traverse = current_node;
			std::vector<qpl::vec2s> path;
			while (traverse) {
				path.push_back(traverse->position);
				traverse = traverse->parent;
			}
			std::ranges::reverse(path);
			return path;
		}

		for (auto& dir : direction) {
			auto check = qpl::vec2is(current_node->position) + dir;

			if (check.x >= 0 && check.x < width && check.y >= 0 && check.y < height) {
				bool valid = maze[check.y][check.x] != 1;
				if (valid) {
					bool found = visited.find(check) != visited.cend();
					if (!found) {
						auto next = std::make_shared<node>(check, current_node);
						queue.push(next);
						visited.insert(check);
					}
				}
			}
		}
	}
	return {};
}

/*
template<typename T, bool diagonal = true> requires (std::is_integral_v<T>)
std::vector<qpl::vec2s> astar(const std::vector<std::vector<T>>& maze, qpl::vec2s start, qpl::vec2s end) {
	if (maze.empty()) {
		return{};
	}
	auto width = qpl::signed_cast(maze[0].size());
	auto height = qpl::signed_cast(maze.size());

	std::vector<std::shared_ptr<astar_node>> queue;
	std::shared_ptr<astar_node> starting_node = std::make_shared<astar_node>(start, nullptr);
	queue.push_back(starting_node);

	std::unordered_set<qpl::vec2s> visited;

	constexpr auto direction = [&]() {
		if constexpr (diagonal) {
			return std::array{
				qpl::vec2is{1, 0}, qpl::vec2is(0, 1), qpl::vec2is(-1, 0), qpl::vec2is(0, -1),
				qpl::vec2is(1, 1), qpl::vec2is(1, -1), qpl::vec2is(-1, 1), qpl::vec2is(-1, -1) };
		}
		else {
			return std::array{ qpl::vec2is{1, 0}, qpl::vec2is(0, 1), qpl::vec2is(-1, 0), qpl::vec2is(0, -1) };
		}
	}();

	while (!queue.empty()) {
		std::shared_ptr<astar_node> current_node = *queue.begin();
		qpl::size current_index = 0u;
		qpl::size ctr = 0u;
		for (auto& element : queue) {
			if (element->f < current_node->f) {
				current_node = element;
				current_index = ctr;
			}
			++ctr;
		}

		visited.insert(current_node->position);
		queue.erase(queue.begin() + current_index);

		if (current_node->position == end) {
			auto traverse = current_node;
			std::vector<qpl::vec2s> path;
			while (traverse) {
				path.push_back(traverse->position);
				traverse = traverse->parent;
			}
			std::ranges::reverse(path);
			return path;
		}

		std::vector<std::shared_ptr<astar_node>> children;
		for (auto& dir : direction) {
			auto check = qpl::vec2is(current_node->position) + dir;

			if (check.x >= 0 && check.x < width && check.y >= 0 && check.y < height) {
				bool valid = maze[check.y][check.x] != 1;
				if (valid) {
					bool found = visited.find(check) != visited.cend();

					bool queue_found = false;
					for (auto& i : queue) {
						if (i->position == check) {
							queue_found = true;
							break;
						}
					}

					if (!found && !queue_found) {
						auto next = std::make_shared<astar_node>(check, current_node);
						children.push_back(next);
					}
				}
			}
		}

		for (auto& child : children) {
			auto x = qpl::signed_cast(child->position.x) - qpl::signed_cast(end.x);
			auto y = qpl::signed_cast(child->position.y) - qpl::signed_cast(end.y);

			child->g = current_node->g + 1u;
			child->h = x * x + y * y;
			child->f = child->g + child->h;

			queue.push_back(child);
		}
	}
	return {};
}
*/

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
	qsf::circles circles;
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

		constexpr auto colors = std::array{ qpl::rgb(140, 140, 140), qpl::rgb(20, 20, 20) };
		for (auto [x, y] : this->dimension.list_possibilities_range()) {

			auto& b = this->before.cells[y][x];
			auto c = maze.cells[y][x];
			if (b != c) {
				b = c;
				this->set_color(x, y, colors[c]);
			}
		}
	}
	void add_path(const std::vector<qpl::vec2s>& path) {
		this->circles.clear();
		for (auto& i : path) {
			qsf::circle circle;
			circle.set_radius(this->cell_dim.x / 2);
			circle.set_color(qpl::rgb::red());
			circle.set_center(this->cell_dim * (i + qpl::vec(0.5, 0.5)));
			this->circles.add_circle(circle);
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
			auto value = perlin.get(qpl::vec(x, y), 0.1, 4);
			this->maze.cells[y][x] = value < 0.45 ? 1 : 0;
		}
	}

	qpl::vec2s maze_size = qpl::vec(1000, 1000);

	void init() override {
		this->maze.create(maze_size);
		this->randomize();
		this->maze_graphic.update(this->maze);

		auto solve = breadth_first_search(this->maze.cells, { 0, 0 }, this->maze_size - 1);
		qpl::println(solve);
		this->maze_graphic.add_path(solve);

		this->bfs.prepare(this->maze.cells, { 0, 0 }, this->maze_size - 1);

		this->view.set_hitbox(*this);
	}

	void updating() override {
		this->update(this->view);

		if (this->event().key_holding(sf::Keyboard::Space)) {
			for (qpl::size i = 0; i < qpl::size_cast(this->steps); ++i) {
				this->bfs.step(this->maze.cells, { 0, 0 }, this->maze_size - 1);
			}
			this->maze_graphic.add_path(this->bfs.path);

			if (this->lock) {
				this->view.set_center(this->bfs.path.back() * maze_graphic::cell_dim);
			}
		}
		if (this->event().key_pressed(sf::Keyboard::A)) {
			this->steps *= 0.75;
		}
		else if (this->event().key_pressed(sf::Keyboard::D)) {
			this->steps *= (4.0 / 3);
		}
		else if (this->event().key_single_pressed(sf::Keyboard::L)) {
			this->lock = !this->lock;
		}
		else if (this->event().key_single_pressed(sf::Keyboard::R)) {
			this->randomize();
			this->bfs.reset();
			this->bfs.prepare(this->maze.cells, { 0, 0 }, this->maze_size - 1);
			this->maze_graphic.update(this->maze);
			this->maze_graphic.add_path(this->bfs.path);
		}
	}
	void drawing() override {
		this->draw(this->maze_graphic, this->view);
	}

	maze maze;
	maze_graphic maze_graphic;
	qsf::view_rectangle view;
	astar_visualized<qpl::size, true> bfs;
	qpl::f64 steps = 5.0;
	bool lock = false;
};

void test() {
	constexpr auto size = qpl::vec(20, 20);
	constexpr auto begin = qpl::vec(0, 0);
	constexpr auto end = size - 1;

	std::vector<std::vector<qpl::size>> maze;
	maze.resize(size.y);
	for (auto& i : maze) {
		i.resize(size.x);
	}

	auto randomize = [&]() {
		for (auto& i : maze) {
			for (auto& i : i) {
				i = qpl::random_b(0.25) ? 1 : 0;
			}
		}
	};

	qpl::clock clock;
	for (qpl::size i = 0u;; ++i) {
		randomize();

		qpl::begin_benchmark("bfs");
		auto solve = breadth_first_search(maze, begin, end);
		qpl::begin_benchmark_end_previous("a*");
		auto astar_solve = qpl::astar(maze, begin, end);
		qpl::end_benchmark();

		if (solve != astar_solve) {
			qpl::println("nequal: ", solve, " vs ", astar_solve);
		}

		if (qpl::get_time_signal(0.5)) {
			qpl::println(qpl::big_number_string(i / clock.elapsed_f()), " / sec");
			qpl::print_benchmark();
		}
	}

}

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