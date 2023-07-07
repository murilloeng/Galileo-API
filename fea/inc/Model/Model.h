#pragma once

//std
#include <string>
#include <functional>

namespace fea
{
	namespace draw
	{
		class Draw;
	}
	namespace mesh
	{
		class Mesh;
	}
	namespace results
	{
		class Results;
	}
	namespace boundary
	{
		class Boundary;
	}
	namespace topology
	{
		class Topology;
	}
	namespace analysis
	{
		class Analysis;
		class Assembler;
	}
}

namespace fea
{
	namespace models
	{
		class Model
		{
			friend class analysis::Analysis;
			friend class analysis::Assembler;

		public:
			//constructors
			Model(std::string = "Model", std::string = "");

			//destructor
			virtual ~Model(void);

			//serialization
			bool load(std::string);
			bool save(std::string = "");

			bool load_results(void);
			bool save_results(void) const;

			//data
			bool saved(void) const;
			void mark(bool = true);

			std::string path(void) const;
			std::string name(void) const;
			std::string path(std::string);
			std::string name(std::string);
			std::string folder(void) const;

			draw::Draw* draw(void) const;
			mesh::Mesh* mesh(void) const;
			results::Results* results(void) const;
			boundary::Boundary* boundary(void) const;
			topology::Topology* topology(void) const;
			analysis::Analysis* analysis(void) const;

			std::function<void(const double*, const char*)> draw_number(void) const;
			std::function<void(const double*, const char*)> draw_number(std::function<void(const double*, const char*)>);

			//draw
			void draw_model(void) const;
			void draw_numbers(void) const;

		protected:
			//header
			void header(void) const;
			void quiter(void) const;

			//serialization
			void load_general(void) const;
			void save_general(void) const;

			//data
			bool m_saved;
			std::string m_path;
			std::string m_name;
			draw::Draw* m_draw;
			mesh::Mesh* m_mesh;
			results::Results* m_results;
			boundary::Boundary* m_boundary;
			topology::Topology* m_topology;
			analysis::Analysis* m_analysis;
			std::function<void(const double*, const char*)> m_draw_numbers;
		};
	}
}