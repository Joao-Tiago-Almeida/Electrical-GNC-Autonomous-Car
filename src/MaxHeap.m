classdef MaxHeap < Heap
%--------------------------------------------------------------------------
% Class:        MaxHeap < Heap (& handle)
%               
% Constructor:  H = MaxHeap(n);
%               H = MaxHeap(n,x0);
%               
% Properties:   (none)
%               
% Methods:                 H.InsertKey(key);
%               sx       = H.Sort();
%               max_node = H.ExtractMax();
%               count    = H.Count();
%               capacity = H.Capacity();
%               bool     = H.IsEmpty();
%               bool     = H.IsFull();
%                          H.Clear();
%                          H.Print();
%               
% Description:  This class implements a max-heap of numeric keys
%               
% Author:       Brian Moore
%               brimoor@umich.edu
%               
% Date:         January 16, 2014
%--------------------------------------------------------------------------
    %
    % Public methods
    %
    methods (Access = public)
        %
        % Constructor
        %
        function this = MaxHeap(varargin)
            %----------------------- Constructor --------------------------
            % Syntax:       H = MaxHeap(n);
            %               H = MaxHeap(n,x0);
            %               
            % Inputs:       n is the maximum number of keys that H can hold
            %               
            %               x0 is a vector (of length <= n) of numeric keys
            %               to insert into the heap during initialization
            %               
            % Description:  Creates a max-heap with capacity n
            %--------------------------------------------------------------
            
            % Call base class constructor
            this = this@Heap(varargin{:});
            
            % Construct the max heap
            this.BuildMaxHeap();
        end
        
        %
        % Insert key
        %
        function InsertKey(this,key)
            %------------------------ InsertKey ---------------------------
            % Syntax:       H.InsertKey(key);
            %               
            % Inputs:       key is a struct
            %               
            % Description:  Inserts key into H
            %--------------------------------------------------------------
            
            this.SetLength(this.k + 1);
            this.x(this.k) = structfun(@(x) -Inf, this.x(this.k), 'UniformOutput', false);
            this.IncreaseKey(this.k,key);
        end
        
        %
        % Sort the heap
        %
        function sx = Sort(this)
            %-------------------------- Sort ------------------------------
            % Syntax:       sx = H.Sort();
            %               
            % Outputs:      sx is a vector that contains the sorted
            %               (ascending order) keys in H
            %               
            % Description:  Returns the sorted values in H
            %--------------------------------------------------------------
            
            % Sort the heap
            nk = this.k; % virtual heap size during sorting procedure
            for i = this.k:-1:2
                this.Swap(1,i);
                nk = nk - 1;
                this.MaxHeapify(1,nk);
            end
            sx = this.x(1:this.k);
            this.x(1:this.k) = flipud(this.x(1:this.k));
        end
                
        %
        % Extract maximum element
        %
        function max_node = ExtractMax(this)
            %------------------------ ExtractMax --------------------------
            % Syntax:       max_node = H.ExtractMax();
            %               
            % Outputs:      max_node is the maximum key in H
            %               
            % Description:  Returns the maximum key in H and extracts it
            %               from the heap
            %--------------------------------------------------------------
            
            if (this.IsEmpty() == true)
                max_node = [];
            else
                this.SetLength(this.k - 1);
                max_node = this.x(1);
                this.x(1) = this.x(this.k + 1);
                this.MaxHeapify(1);
            end
        end
        
        %
        % Update Element
        %
        function UpdateElement(this,key)
            %----------------------- UpdateElement ------------------------
            % Syntax:       max_node = H.UpdateElement();
            %               
            % Inputs:       key is a struct
            %
            % Description:  Updates the position of a node
            %--------------------------------------------------------------
            
            if (this.IsEmpty() == true)
                InsertKey(this,key);
            else
                % search in the heap for the node
                for i = 1:this.k
                    if(key.index ~= this(i).index); continue; end
                    
                    % found node in the matrix
                    if(key.cost > this(i).cost)
                        IncreaseKey(this,i,key)
                    end
                    % update key
                    return
                end
            end
        end
        
        %
        % Prints the heap
        %
        function Print(this)
            %-------------------------- Print -----------------------------
            % Syntax:       H.Print();
            %               
            % Outputs:      
            %               
            % Description:  Prints the heap in the Command Window
            %--------------------------------------------------------------
            size_level = 1;
            count=0;
            for i=1:this.k
                fprintf(sprintf('%d. ',log2(size_level)))
                
                for j=1:size_level
                    
                    % break neighbours in lines
                    count=count+1;
                    if(count>2)
                        fprintf('\n   ');
                        count=1;
                    end
                    
                    idx = size_level+j-1;
                    if(idx>this.k);fprintf('\n');return;end
                    S = sprintf('(%d - %.5f)\t',this.x(idx).previous,this.x(idx).cost); 
                    fprintf(S);
                end
                fprintf('\n\n');
                size_level = size_level*2;
                count=0;
            end
        end
    end
    
    %
    % Private methods
    %
    methods (Access = private)
        %
        % Increase key at index i
        %
        function IncreaseKey(this,i,key)
            if (i > this.k)
                % Index overflow error
                MaxHeap.IndexOverflowError();
            elseif (key.cost < this.x(i).cost)
                % Increase key error
                MaxHeap.IncreaseKeyError();
            end
            this.x(i) = key;
            while ((i > 1) && (this.x(Heap.parent(i)).cost < this.x(i).cost))
                this.Swap(i,Heap.parent(i));
                i = Heap.parent(i);
            end
        end
        
        %
        % Build the max heap
        %
        function BuildMaxHeap(this)
            for i = floor(this.k / 2):-1:1
                this.MaxHeapify(i);
            end
        end
        
        %
        % Maintain the max heap property at a given node
        %
        function MaxHeapify(this,i,size)
            % Parse inputs
            if (nargin < 3)
                size = this.k;
            end
            
            ll = Heap.left(i);
            rr = Heap.right(i);
            if ((ll <= size) && (this.x(ll).cost > this.x(i).cost))
                largest = ll;
            else
                largest = i;
            end
            if ((rr <= size) && (this.x(rr).cost > this.x(largest).cost))
                largest = rr;
            end
            if (largest ~= i)
                this.Swap(i,largest);
                this.MaxHeapify(largest,size);
            end
        end
    end
    
    %
    % Private static methods
    %
    methods (Access = private, Static = true)
        %
        % Increase key error
        %
        function IncreaseKeyError()
            error('You can only increase keys in MaxHeap');
        end
        
        %
        % Index overflow error
        %
        function IndexOverflowError()
            error('MaxHeap index overflow');
        end
    end
end