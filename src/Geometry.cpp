#include <cmath>
#include <list>
#include <fstream>
#include <memory>

using namespace std;

struct Vec3D {
    double X, Y, Z;

    double Length() const;
};

struct Block {
    Vec3D Min;
    Vec3D Max;
};

double Vec3D::Length() const {
    return sqrt(X * X + Y * Y + Z * Z);
};

struct ISolidBody {
    virtual bool IsInside(Vec3D coord) = 0;
    virtual Vec3D ExtentsLow() = 0;
    virtual Vec3D ExtentsHi() = 0;
};
typedef std::shared_ptr<ISolidBody> ISolidBodyPtr;

bool IsCompleteInside(Block block, ISolidBodyPtr solid) {
    double size_x = block.Max.X - block.Min.X;
    double size_y = block.Max.Y - block.Min.Y;
    double size_z = block.Max.Z - block.Min.Z;
    unsigned step_count = 10;
    double step_x = size_x / step_count;
    double step_y = size_y / step_count;
    double step_z = size_z / step_count;
    for (double x = block.Min.X; x <= block.Max.X; x += step_x) {
        for (double y = block.Min.Y; y <= block.Max.Y; y += step_y) {
            for (double z = block.Min.Z; z <= block.Max.Z; z += step_z) {
                Vec3D cur_point;
                cur_point.X = x;
                cur_point.Y = y;
                cur_point.Z = z;
                if (!solid->IsInside(cur_point))
                {
                    return false;
                }
            }
        }
    }
    return true;
}

list<Block> g_blocks;

void TessellateBlock(Block& block, ISolidBodyPtr solid) {
	Vec3D diagonal;
	diagonal.X = block.Max.X - block.Min.X;
	diagonal.Y = block.Max.Y - block.Min.Y;
	diagonal.Z = block.Max.Z - block.Min.Z;
	if (diagonal.Length() >= 0.15)
	{
		if (IsCompleteInside(block, solid))
		{
			g_blocks.push_back(block);
		}
		else
		{
			const double middle_x = (block.Max.X + block.Min.X) / 2;
			const double middle_y = (block.Max.Y + block.Min.Y) / 2;
			const double middle_z = (block.Max.Z + block.Min.Z) / 2;
			const double min_x = block.Min.X;
			const double min_y = block.Min.Y;
			const double min_z = block.Min.Z;
			const double max_x = block.Max.X;
			const double max_y = block.Max.Y;
			const double max_z = block.Max.Z;

			Block new_blocks[8];
			new_blocks[0].Min.X = min_x;
			new_blocks[0].Min.Y = middle_y;
			new_blocks[0].Min.Z = min_z;
			new_blocks[0].Max.X = middle_x;
			new_blocks[0].Max.Y = max_y;
			new_blocks[0].Max.Z = middle_z;

			new_blocks[1].Min.X = middle_x;
			new_blocks[1].Min.Y = middle_y;
			new_blocks[1].Min.Z = min_z;
			new_blocks[1].Max.X = max_x;
			new_blocks[1].Max.Y = max_y;
			new_blocks[1].Max.Z = middle_z;

			new_blocks[2].Min.X = middle_x;
			new_blocks[2].Min.Y = middle_y;
			new_blocks[2].Min.Z = middle_z;
			new_blocks[2].Max.X = max_x;
			new_blocks[2].Max.Y = max_y;
			new_blocks[2].Max.Z = max_z;

			new_blocks[3].Min.X = min_x;
			new_blocks[3].Min.Y = middle_y;
			new_blocks[3].Min.Z = middle_z;
			new_blocks[3].Max.X = middle_x;
			new_blocks[3].Max.Y = max_y;
			new_blocks[3].Max.Z = max_z;

			new_blocks[4].Min.X = min_x;
			new_blocks[4].Min.Y = min_y;
			new_blocks[4].Min.Z = min_z;
			new_blocks[4].Max.X = middle_x;
			new_blocks[4].Max.Y = middle_y;
			new_blocks[4].Max.Z = middle_z;

			new_blocks[5].Min.X = middle_x;
			new_blocks[5].Min.Y = min_y;
			new_blocks[5].Min.Z = min_z;
			new_blocks[5].Max.X = max_x;
			new_blocks[5].Max.Y = middle_y;
			new_blocks[5].Max.Z = middle_z;

			new_blocks[6].Min.X = middle_x;
			new_blocks[6].Min.Y = min_y;
			new_blocks[6].Min.Z = middle_z;
			new_blocks[6].Max.X = max_x;
			new_blocks[6].Max.Y = middle_y;
			new_blocks[6].Max.Z = max_z;

			new_blocks[7].Min.X = min_x;
			new_blocks[7].Min.Y = min_y;
			new_blocks[7].Min.Z = middle_z;
			new_blocks[7].Max.X = middle_x;
			new_blocks[7].Max.Y = middle_y;
			new_blocks[7].Max.Z = max_z;

			for (int count = 0; count <= 7; count++)
			{
				TessellateBlock(new_blocks[count], solid);
			}
		}
	}
};

void Tessellate(ISolidBodyPtr solid) {
    Block cur_block;
    cur_block.Min = solid->ExtentsLow();
    cur_block.Max = solid->ExtentsHi();
    TessellateBlock(cur_block, solid);
};

struct Cube : public ISolidBody {
    double Size;

    virtual bool IsInside(Vec3D coord) {
        if (coord.X >= 0 && coord.X < Size &&
            coord.Y >= 0 && coord.Y < Size &&
            coord.Z >= 0 && coord.Z < Size) 
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    virtual Vec3D ExtentsLow()
    {
        Vec3D MinCoordinate;
        MinCoordinate.X = 0;
        MinCoordinate.Y = 0;
        MinCoordinate.Z = 0;
        return MinCoordinate;
    }
    virtual Vec3D ExtentsHi()
    {
        Vec3D MaxCoordinate;
        MaxCoordinate.X = Size;
        MaxCoordinate.Y = Size;
        MaxCoordinate.Z = Size;
        return MaxCoordinate;
    }

};
typedef std::shared_ptr<Cube> CubePtr;

struct Sphere : public ISolidBody {
    double Radius;
    
    virtual bool IsInside(Vec3D coord) 
    {
        if (coord.Length() <= Radius) {
            return true;
        }
        else {
            return false;
        }
    }
    virtual Vec3D ExtentsLow()
    {
        Vec3D MinCoordinate;
        MinCoordinate.X = -Radius;
        MinCoordinate.Y = -Radius;
        MinCoordinate.Z = -Radius;
        return MinCoordinate;
    }
    virtual Vec3D ExtentsHi()
    {
        Vec3D MaxCoordinate;
        MaxCoordinate.X = Radius;
        MaxCoordinate.Y = Radius;
        MaxCoordinate.Z = Radius;
        return MaxCoordinate;
    }
};
typedef std::shared_ptr<Sphere> SpherePtr;

bool CheckTurn(Vec3D coord, double Radius, double Height)
{
    if ((sqrt(coord.X * coord.X + coord.Z * coord.Z) <= Radius) && (coord.Y <= Height) && (coord.Y >= 0))
    {
        return true;
    }
    else
    {
        return false;
    }
}

struct Cylinder : public ISolidBody {
    double Radius;
    double Height;
    bool Turn;

    virtual bool IsInside(Vec3D coord)
    {
        if (Turn == true)
        {
            Vec3D TempTurn = coord;
            TempTurn.X = coord.Y;
            TempTurn.Y = coord.X;
            return CheckTurn(TempTurn,Radius,Height);
        }
        else
        {
            Vec3D TempTurn = coord;
            TempTurn.X = coord.X;
            TempTurn.Y = coord.Y;
            return CheckTurn(TempTurn, Radius, Height);
        }
    }

    virtual Vec3D ExtentsLow()
    {
        Vec3D MinCoordinate;
        MinCoordinate.X = -Radius;
        MinCoordinate.Y = 0;
        MinCoordinate.Z = -Radius;
        return MinCoordinate;
    }

    virtual Vec3D ExtentsHi()
    {
        Vec3D MaxCoordinate;
        MaxCoordinate.X = Radius;
        MaxCoordinate.Y = Height;
        MaxCoordinate.Z = Radius;
        return MaxCoordinate;
    }
};
typedef std::shared_ptr<Cylinder> CylinderPtr;


struct Transformation : public ISolidBody {
    Vec3D Offset;
    ISolidBodyPtr Solid;

    virtual bool IsInside(Vec3D coord) {
        Vec3D transformed_coord;
        transformed_coord.X = coord.X - Offset.X;
        transformed_coord.Y = coord.Y - Offset.Y;
        transformed_coord.Z = coord.Z - Offset.Z;
        return Solid->IsInside(transformed_coord);
    }
    virtual Vec3D ExtentsLow()
    {
        Vec3D MinCoordinate;
        MinCoordinate = Solid->ExtentsLow();
        MinCoordinate.X +=Offset.X;
        MinCoordinate.Y +=Offset.Y;
        MinCoordinate.Z +=Offset.Z;
        return MinCoordinate;
    }
    virtual Vec3D ExtentsHi()
    {
        Vec3D MaxCoordinate;
        MaxCoordinate = Solid->ExtentsHi();
        MaxCoordinate.X += Offset.X;
        MaxCoordinate.Y += Offset.Y;
        MaxCoordinate.Z += Offset.Z;
        return MaxCoordinate;
    }
};
typedef std::shared_ptr<Transformation> TransformationPtr;

double SearchMinCoordinate(double A, double B)
{
    if (A <= B)
    {
        return A;
    }
    else
    {
        return B;
    }
}

double SearchMaxCoordinate(double A, double B)
{
    if (A >= B)
    {
        return A;
    }
    else
    {
        return B;
    }
}

Vec3D SearchExtentsLow(ISolidBodyPtr A,ISolidBodyPtr B)
{
    Vec3D MinCoordinateA, MinCoordinateB, MinCoordinate;
    MinCoordinateA = A->ExtentsLow();
    MinCoordinateB = B->ExtentsLow();
    MinCoordinate.X = SearchMinCoordinate(MinCoordinateA.X, MinCoordinateB.X);
    MinCoordinate.Y = SearchMinCoordinate(MinCoordinateA.Y, MinCoordinateB.Y);
    MinCoordinate.Z = SearchMinCoordinate(MinCoordinateA.Z, MinCoordinateB.Z);
    return MinCoordinate;
}

Vec3D SearchExtentsHi(ISolidBodyPtr A, ISolidBodyPtr B)
{
    Vec3D MaxCoordinateA, MaxCoordinateB, MaxCoordinate;
    MaxCoordinateA = A->ExtentsHi();
    MaxCoordinateB = B->ExtentsHi();
    MaxCoordinate.X = SearchMaxCoordinate(MaxCoordinateA.X, MaxCoordinateB.X);
    MaxCoordinate.Y = SearchMaxCoordinate(MaxCoordinateA.Y, MaxCoordinateB.Y);
    MaxCoordinate.Z = SearchMaxCoordinate(MaxCoordinateA.Z, MaxCoordinateB.Z);
    return MaxCoordinate;
}

struct Union : public ISolidBody {
    ISolidBodyPtr A;
    ISolidBodyPtr B;

    virtual bool IsInside(Vec3D coord) {
        if ((A->IsInside(coord)) || (B->IsInside(coord))) {
            return true;
        }
        else {
            return false;
        }
    }
    virtual Vec3D ExtentsLow()
    {
        Vec3D MinCoordinate = SearchExtentsLow(A, B);
        return MinCoordinate;
    }
    virtual Vec3D ExtentsHi()
    {
        Vec3D MaxCoordinate = SearchExtentsHi(A, B);
        return MaxCoordinate;
    }
};
typedef std::shared_ptr<Union> UnionPtr;

struct Intersection : public ISolidBody {
    ISolidBodyPtr A;
    ISolidBodyPtr B;

    virtual bool IsInside(Vec3D coord) {
        if (A->IsInside(coord) && B->IsInside(coord)) {
            return true;
        }
        else {
            return false;
        }
    }
    virtual Vec3D ExtentsLow()
    {
        Vec3D MinCoordinate = SearchExtentsLow(A, B);
        return MinCoordinate;
    }
    virtual Vec3D ExtentsHi()
    {
        Vec3D MaxCoordinate = SearchExtentsHi(A, B);
        return MaxCoordinate;
    }
};
typedef std::shared_ptr<Intersection> IntersectionPtr;

struct Difference : public ISolidBody {
    ISolidBodyPtr A;
    ISolidBodyPtr B;

    virtual bool IsInside(Vec3D coord) {
        if (A->IsInside(coord) && !B->IsInside(coord)) {
            return true;
        }
        else {
            return false;
        }
    }
    virtual Vec3D ExtentsLow()
    {
        Vec3D MinCoordinate = SearchExtentsLow(A, B);
        return MinCoordinate;
    }
    virtual Vec3D ExtentsHi()
    {
        Vec3D MaxCoordinate = SearchExtentsHi(A, B);
        return MaxCoordinate;
    }
};
typedef std::shared_ptr<Difference> DifferencePtr;

void TriangulationDown(ofstream& file)
{
	int count = 0;
	Block Temp;
	for (Block Coordinate : g_blocks)
	{
        Temp.Min.Z = Coordinate.Min.Z;
        count = 0;
		while (count <= 1)
		{
            if (count == 1)
            {
                Temp.Min.Z = Coordinate.Max.Z;
            }
			file << "facet normal  0.0  0.0  0.0" << endl;
			file << "outer loop" << endl;

			file << "vertex " << Coordinate.Min.X << " " << Temp.Min.Z << " " << Coordinate.Min.Y << endl;
			file << "vertex " << Coordinate.Max.X << " " << Temp.Min.Z << " " << Coordinate.Min.Y << endl;
			file << "vertex " << Coordinate.Max.X << " " << Temp.Min.Z << " " << Coordinate.Max.Y << endl;
			file << "endloop" << endl;
			file << "endfacet" << endl;

			file << "facet normal 0.0  0.0  0.0" << endl;
			file << "outer loop" << endl;
			file << "vertex " << Coordinate.Min.X << " " << Temp.Min.Z << " " << Coordinate.Min.Y << endl;
			file << "vertex " << Coordinate.Min.X << " " << Temp.Min.Z << " " << Coordinate.Max.Y << endl;
			file << "vertex " << Coordinate.Max.X << " " << Temp.Min.Z << " " << Coordinate.Max.Y << endl;
			file << "endloop" << endl;
			file << "endfacet" << endl;
            count++;
		}
	}
}

void TriangulationHorizontal(ofstream& file)
{
    int count = 0;
    Block Temp;
    for (Block Coordinate : g_blocks)
    {
        Temp.Min.X = Coordinate.Min.X;
        count = 0;
        while (count <= 1)
        {
            if (count == 1)
            {
                Temp.Min.X = Coordinate.Max.X;
            }
            file << "facet normal  0.0  0.0  0.0" << endl;
            file << "outer loop" << endl;

            file << "vertex " << Temp.Min.X << " " << Coordinate.Max.Z << " " << Coordinate.Min.Y << endl;
            file << "vertex " << Temp.Min.X << " " << Coordinate.Min.Z << " " << Coordinate.Min.Y << endl;
            file << "vertex " << Temp.Min.X << " " << Coordinate.Min.Z << " " << Coordinate.Max.Y << endl;
            file << "endloop" << endl;
            file << "endfacet" << endl;

            file << "facet normal 0.0  0.0  0.0" << endl;
            file << "outer loop" << endl;
            file << "vertex " << Temp.Min.X << " " << Coordinate.Max.Z << " " << Coordinate.Min.Y << endl;
            file << "vertex " << Temp.Min.X << " " << Coordinate.Max.Z << " " << Coordinate.Max.Y << endl;
            file << "vertex " << Temp.Min.X << " " << Coordinate.Min.Z << " " << Coordinate.Max.Y << endl;
            file << "endloop" << endl;
            file << "endfacet" << endl;
            count++;
        }
    }
}

void TriangulationVertical(ofstream& file)
{
    int count = 0;
    Block Temp;
    for (Block Coordinate : g_blocks)
    {
        Temp.Min.Y = Coordinate.Min.Y;
        count = 0;
        while (count <= 1)
        {
            if (count == 1)
            {
                Temp.Min.Y = Coordinate.Max.Y;
            }
            file << "facet normal  0.0  0.0  0.0" << endl;
            file << "outer loop" << endl;

            file << "vertex " << Coordinate.Min.X << " " << Coordinate.Max.Z << " " << Temp.Min.Y << endl;
            file << "vertex " << Coordinate.Min.X << " " << Coordinate.Min.Z << " " << Temp.Min.Y << endl;
            file << "vertex " << Coordinate.Max.X << " " << Coordinate.Min.Z << " " << Temp.Min.Y << endl;
            file << "endloop" << endl;
            file << "endfacet" << endl;

            file << "facet normal 0.0  0.0  0.0" << endl;
            file << "outer loop" << endl;
            file << "vertex " << Coordinate.Min.X << " " << Coordinate.Max.Z << " " << Temp.Min.Y << endl;
            file << "vertex " << Coordinate.Max.X << " " << Coordinate.Max.Z << " " << Temp.Min.Y << endl;
            file << "vertex " << Coordinate.Max.X << " " << Coordinate.Min.Z << " " << Temp.Min.Y << endl;
            file << "endloop" << endl;
            file << "endfacet" << endl;
            count++;
        }
    }
}

void Triangulation(ofstream& file)
{
    TriangulationDown(file);
    TriangulationHorizontal(file);
    TriangulationVertical(file);
}

void ExitFile(ofstream& file)
{
    file << "endsolid";
    file.close();
}

int main() {
    CylinderPtr cy1 = std::make_shared<Cylinder>();
    cy1 -> Radius = 1;
    cy1 -> Height = 2;
    cy1 -> Turn = false;
    CubePtr cube1 = std::make_shared<Cube>();;
    cube1 -> Size = 3;
    TransformationPtr trCube = std::make_shared<Transformation>();
    trCube -> Solid = cube1;
    trCube -> Offset.X = -1.5;
    trCube -> Offset.Y = 2;
    trCube -> Offset.Z = -1.5;
    UnionPtr pl1 = std::make_shared<Union>();
    pl1 -> A = trCube;
    pl1 -> B = cy1;
    CylinderPtr cy2 = std::make_shared<Cylinder>();
    cy2 -> Radius = 1.5;
    cy2 -> Height = 3;
    cy2 -> Turn = true;
    TransformationPtr trCylinder2 = std:: make_shared<Transformation>();
    trCylinder2 -> Solid = cy2;
    trCylinder2 ->Offset.X = -1.5;
    trCylinder2 -> Offset.Y = 5;
    trCylinder2 -> Offset.Z = 0;
    UnionPtr pl2 = std::make_shared<Union>();
    pl2 -> A = pl1;
    pl2 -> B = trCylinder2;
    CylinderPtr cy3 =std::make_shared<Cylinder>();
    cy3 -> Radius = 0.7;
    cy3 -> Height = 3;
    cy3 -> Turn = true;
    TransformationPtr trCylinder3 = std::make_shared<Transformation>();
    trCylinder3 -> Solid = cy3;
    trCylinder3 -> Offset.X = -1.5;
    trCylinder3 -> Offset.Y = 5;
    trCylinder3 -> Offset.Z = 0;
    DifferencePtr dif = std::make_shared<Difference>();
    dif -> A = pl2;
    dif -> B = trCylinder3;

    Tessellate(dif);
    ofstream file("cppstudio.stl");
    file << "solid cube_corner" << endl;
    Triangulation(file);
    ExitFile(file);
    return 0;
}
